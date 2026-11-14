// exskel - Nologo ESP32C3 Super Mini + 0.91" 128x32 SSD1306 OLED (I2C) + PMW3360 -> BLE mouse
//
// Wiring (board default pins):
//   OLED    VCC-3V3  GND-GND  SDA-GPIO8  SCL-GPIO9
//   PMW3360 VDD-3V3 (module has 2.0 V LDO)  VDDIO-3V3  GND-GND  SCLK-GPIO4  MISO-GPIO5
//           MOSI-GPIO6  NCS-GPIO7  MOT-GPIO3  NRESET open or 3V3 (internal pull-up)
//   Buttons left: GPIO1 -> button -> GND   right: GPIO10 -> button -> GND
//           middle: GPIO21 -> button -> GND
//           mode: GPIO2 -> button -> GND  (strapping pin: don't hold it while powering on)
//   Wheel   TTC encoder  A-GPIO0  C (middle pin)-GND  B-GPIO20
//
// The PMW3360 is detected at boot. Without it the OLED runs a test screen.
//
// Mode button: normal -> BT channel menu -> CPI menu -> normal (cancel).
//   In a menu: wheel selects, left confirms, right cancels, 8 s idle cancels.
//   Menu clicks/scrolls never reach the host; the cursor still moves.
//   CPI previews live while scrolling; cancel restores it. Confirmed CPI and channel live in NVS.
//   BT channel: each channel is its own BLE identity, one host per channel. Switching reboots
//   into the new channel. To pair a new host on a channel, select it and pair from the host.
//   Hold right 3 s on a highlighted channel: forget its host (bond keys + name). Also remove the
//   mouse on that host, or it keeps retrying with its old keys.
//   Channel lock: once a channel has a host, any other host that connects is dropped and its bond
//   deleted. Clear the channel (above) to pair a different host.
//
// Tasks:
//   loop()   (prio 2) sensor + BLE. Sleeps until MOT, a button or the wheel fires, reads, sends the
//                     report, and only when there is new data hands a snapshot to the OLED task.
//   oledTask (prio 1) redraws the OLED from the latest snapshot. Lower priority, so the
//                     ~25 ms I2C frame transfer never delays the cursor. A frame is only sent
//                     when its pixels changed.

#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PMW3360.h>
#include <esp_pm.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <hal/gpio_ll.h>

// Serial log (boot info, BLE conn params, once-a-second motion stats)
constexpr bool SERIAL_LOG = false;
#define LOGF(...) do { if (SERIAL_LOG) Serial.printf(__VA_ARGS__); } while (0)

#include "ble_hid_mouse.h"

// ---------------- Config ----------------
constexpr int PIN_I2C_SDA  = 8;
constexpr int PIN_I2C_SCL  = 9;
constexpr int PIN_SPI_SCK  = 4;
constexpr int PIN_SPI_MISO = 5;
constexpr int PIN_SPI_MOSI = 6;
constexpr int PIN_PMW_CS   = 7;
constexpr int PIN_PMW_MOT  = 3;      // MOTION, active low: sensor pulls it low when it has moved

constexpr uint16_t CPI_PRESETS[] = {400, 800, 1200, 1600, 2400, 3200, 6400};
constexpr uint8_t CPI_DEFAULT_IDX = 3;  // 1600 until something is saved
constexpr bool INVERT_X = false;     // both inverted = sensor mounted rotated 180°
constexpr bool INVERT_Y = false;     // flipped back after final assembly (2026-11-11)
constexpr bool SWAP_XY  = false;

constexpr int PIN_BTN_LEFT  = 1;     // pin -> button -> GND (internal pull-up)
constexpr int PIN_BTN_RIGHT = 10;
constexpr int PIN_BTN_MIDDLE = 21;
constexpr int PIN_BTN_MODE  = 2;

constexpr int PIN_WHEEL_A = 0;       // encoder side pins, middle pin to GND (internal pull-ups)
constexpr int PIN_WHEEL_B = 20;
constexpr int WHEEL_STEPS_PER_DETENT = 2;  // encoder transitions per click; set to 4 if one click scrolls twice
constexpr bool INVERT_WHEEL = true;
constexpr uint32_t WHEEL_DEBOUNCE_US = 500;  // A/B state must hold this long to count (bounce ~<1 ms)

const char* BLE_NAME = "exskel Mouse";     // channel 2/3 advertise as "exskel Mouse 2"/"... 3"
constexpr uint8_t BLE_CHANNELS = 3;
constexpr uint32_t MOUSE_PERIOD_MS = 8;    // read rate while moving, until the BLE interval is known
constexpr uint32_t OLED_TIMEOUT_MS = 10000; // OLED sleeps after this long without input
constexpr uint32_t MENU_TIMEOUT_MS = 8000;  // menu cancels after this long without wheel/button
constexpr uint32_t CLEAR_HOLD_MS = 3000;    // BT menu: hold right this long to clear the channel
constexpr uint32_t SLEEP_AFTER_MS = 5 * 60 * 1000;  // no input this long -> deep sleep (wake: move / left click)
constexpr bool DEBUG_MODE = false;         // true: OLED never sleeps and shows dx/dy, SQUAL, wheel, buttons
// ----------------------------------------

constexpr uint8_t CPI_COUNT = sizeof(CPI_PRESETS) / sizeof(CPI_PRESETS[0]);
constexpr uint8_t BTN_MODE = 0x08;   // internal only, masked off before sending to the host
enum : uint8_t { MENU_NONE, MENU_BT, MENU_CPI };

Adafruit_SSD1306 display(128, 32, &Wire, -1);
PMW3360 sensor;
BleHidMouse mouse;
Preferences prefs;

bool oledOk = false;
uint8_t oledAddr = 0;
bool sensorOk = false;

uint8_t bleChannel = 0;              // active channel, 0-based (shown as 1..3)
uint8_t cpiIdx = CPI_DEFAULT_IDX;    // confirmed CPI preset
char chName[BLE_CHANNELS][32] = {};  // last host name seen on each channel ("" = never paired)
ble_addr_t chAddr[BLE_CHANNELS] = {}; // bonded host's identity address per channel
bool chHasAddr[BLE_CHANNELS] = {};

// loop() -> oledTask hand-off. Length-1 queue + xQueueOverwrite = "latest value" mailbox.
// Totals are running sums, so a skipped snapshot loses no movement.
struct Snapshot {
  int32_t  totalX, totalY, totalWheel;
  uint32_t reports;
  uint8_t  squal;
  bool     surface;
  uint8_t  buttons;
  uint8_t  menu, sel;   // open menu and highlighted item
  bool     switching;   // BT channel confirmed, about to reboot
  uint8_t  clearHold;   // BT menu: seconds left of the right-button hold (0 = not holding)
};
QueueHandle_t oledQueue;

TaskHandle_t loopHandle;  // loopTask, woken by MOT / button / wheel interrupts

// MOT / button / wheel change -> wake loop()
void IRAM_ATTR wakeLoop() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(loopHandle, &woken);
  portYIELD_FROM_ISR(woken);
}

// Light sleep only wakes on GPIO *levels*, so every input uses a level interrupt armed for the
// opposite of the pin's current level. Each ISR re-arms it -> fires (and wakes the chip) on every
// change, like CHANGE. If the pin flips again meanwhile, the new level fires at once: nothing lost.
static inline void IRAM_ATTR rearm(int pin, uint32_t in) {
  gpio_ll_set_intr_type(&GPIO, pin, ((in >> pin) & 1) ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
}

void IRAM_ATTR inputIsr(void* pin) {  // MOT and buttons
  rearm((int)(intptr_t)pin, REG_READ(GPIO_IN_REG));
  wakeLoop();
}

// Quadrature decode of debounced A/B states (see wheelAccept).
volatile int32_t wheelRaw = 0;

// Debounce: the contacts bounce for up to ~1 ms per step (bursts of edges 2-5 us apart, and the
// common pin lifting makes A and B jump to 11 together). A state only counts once it has held
// for WHEEL_DEBOUNCE_US: checked on the next edge (ISR) or by loop() once the wheel goes quiet.
static uint8_t wheelStable = 0, wheelCand = 0;
static uint32_t wheelCandUs = 0;
portMUX_TYPE wheelMux = portMUX_INITIALIZER_UNLOCKED;

// Caller holds wheelMux. Counts the candidate state once it is stable.
static void IRAM_ATTR wheelAccept(uint32_t nowUs) {
  static const int8_t table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  if (wheelCand == wheelStable || nowUs - wheelCandUs < WHEEL_DEBOUNCE_US) return;
  wheelRaw += table[(wheelStable << 2) | wheelCand];  // a 2-state jump (missed step) counts 0
  wheelStable = wheelCand;
}

void IRAM_ATTR wheelIsr() {
  uint32_t in = REG_READ(GPIO_IN_REG);
  rearm(PIN_WHEEL_A, in);
  rearm(PIN_WHEEL_B, in);
  uint8_t ab = (((in >> PIN_WHEEL_A) & 1) << 1) | ((in >> PIN_WHEEL_B) & 1);
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&wheelMux);
  wheelAccept(now);  // the previous candidate held long enough?
  wheelCand = ab;
  wheelCandUs = now;
  portEXIT_CRITICAL_ISR(&wheelMux);
  wakeLoop();
}

// loop(): accept the final state after the wheel stops. True while a change is still settling.
static bool wheelSettle() {
  portENTER_CRITICAL(&wheelMux);
  wheelAccept(micros());
  bool pending = wheelCand != wheelStable;
  portEXIT_CRITICAL(&wheelMux);
  return pending;
}

static uint8_t readButtons() {
  uint8_t b = 0;
  if (digitalRead(PIN_BTN_LEFT) == LOW)   b |= MOUSE_LEFT;
  if (digitalRead(PIN_BTN_RIGHT) == LOW)  b |= MOUSE_RIGHT;
  if (digitalRead(PIN_BTN_MIDDLE) == LOW) b |= MOUSE_MIDDLE;
  if (digitalRead(PIN_BTN_MODE) == LOW)   b |= BTN_MODE;
  return b;
}

static void nameKey(char* key, uint8_t ch) { key[0] = 'n'; key[1] = '0' + ch; key[2] = 0; }
static void addrKey(char* key, uint8_t ch) { key[0] = 'a'; key[1] = '0' + ch; key[2] = 0; }

// Forget channel ch's host: delete its bond keys, saved address and name
static void clearChannel(uint8_t ch) {
  if (chHasAddr[ch]) BleHidMouse::unbond(chAddr[ch]);
  if (ch == bleChannel) { mouse.setLockedHost(nullptr); mouse.disconnect(); }  // open for a new host
  chHasAddr[ch] = false;
  chName[ch][0] = 0;
  char key[3];
  addrKey(key, ch); prefs.remove(key);
  nameKey(key, ch); prefs.remove(key);
  LOGF("BT channel %u cleared\n", ch + 1);
}

static void applyCpi(uint8_t idx) {
  if (sensorOk) sensor.setCPI(CPI_PRESETS[idx]);
}

void oledTask(void*) {
  Snapshot s = {};
  int32_t frameX = 0, frameY = 0, secX = 0, secY = 0, secW = 0;
  uint32_t lastRate = 0, lastReports = 0, rate = 0;
  bool moving = false;
  bool oledOn = true;
  uint32_t lastActivity = millis(), onSince = lastActivity;  // onSince: start of the 10 s show window

  for (;;) {
    // Wait for new data from loop(). On timeout redraw with the last snapshot:
    // 50 ms after motion so dx/dy drop to 0, else once a second for the BLE status.
    // While the OLED sleeps, just wait for data. Without a sensor keep redrawing
    // so the test-mode bar moves.
    TickType_t wait = !sensorOk ? pdMS_TO_TICKS(20)
                    : !oledOn   ? portMAX_DELAY
                    : pdMS_TO_TICKS(moving ? 50 : 1000);
    bool fresh = xQueueReceive(oledQueue, &s, wait) == pdTRUE;  // on timeout s keeps the previous value
    uint32_t now = millis();
    if (fresh) {
      if (now - lastActivity >= OLED_TIMEOUT_MS) onSince = now;  // input after a pause: show again
      lastActivity = now;
    }

    int32_t fdx = s.totalX - frameX, fdy = s.totalY - frameY;
    frameX = s.totalX; frameY = s.totalY;
    moving = fdx || fdy;

    if (now - lastRate >= 1000) {
      rate = s.reports - lastReports;
      lastReports = s.reports;
      lastRate = now;
      if (s.totalX != secX || s.totalY != secY || s.totalWheel != secW)  // Serial only while moving
        LOGF("BLE:%s itv:%u.%02ums dx/s:%ld dy/s:%ld wheel/s:%ld SQUAL:%u surf:%d rep/s:%u\n",
                      mouse.isConnected() ? "conn" : "adv",
                      mouse.connInterval() * 125 / 100, mouse.connInterval() * 125 % 100,
                      (long)(s.totalX - secX), (long)(s.totalY - secY), (long)(s.totalWheel - secW),
                      s.squal, s.surface, rate);
      secX = s.totalX; secY = s.totalY; secW = s.totalWheel;
    }

    if (!oledOk) continue;

    // Panel shows for OLED_TIMEOUT_MS after input resumes (or a menu closes), then sleeps
    // (0xAE, ~10 uA, RAM kept) even while the mouse keeps moving. Mode button opens a menu = shows.
    if (s.menu != MENU_NONE || s.switching) onSince = now;
    if (!DEBUG_MODE && sensorOk && now - onSince >= OLED_TIMEOUT_MS) {
      if (oledOn) { display.ssd1306_command(SSD1306_DISPLAYOFF); oledOn = false; }
      continue;
    }
    if (!oledOn) { display.ssd1306_command(SSD1306_DISPLAYON); oledOn = true; }

    // 128x32 = 4 lines x 21 chars
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    if (s.switching) {
      display.printf("Switching to BT %u\n\nreconnecting...", s.sel + 1);
    } else if (s.menu == MENU_BT) {
      if (s.clearHold) display.printf("Clear BT%u? hold %u\n", s.sel + 1, s.clearHold);
      else display.println("BT channel  L:OK R:X");
      for (uint8_t i = 0; i < BLE_CHANNELS; i++)  // > = highlighted, * = active
        display.printf("%c%u%c%s\n", i == s.sel ? '>' : ' ', i + 1, i == bleChannel ? '*' : ' ',
                       chName[i][0] ? chName[i] : "(empty)");
    } else if (s.menu == MENU_CPI) {
      display.println("CPI         L:OK R:X");
      display.setTextSize(2);
      display.setCursor(16, 13);
      display.printf("%c %4u %c", s.sel > 0 ? '<' : ' ', CPI_PRESETS[s.sel],
                     s.sel < CPI_COUNT - 1 ? '>' : ' ');
    } else {
      display.printf("BT%u %-4s  CPI %u\n", bleChannel + 1, mouse.isConnected() ? "CONN" : "adv",
                     CPI_PRESETS[cpiIdx]);
      if (sensorOk && !DEBUG_MODE) {
        // host name runs off the right edge (text wrap is off)
        display.printf("Host:%s", mouse.isConnected() ? mouse.peerName() : "-");
      } else if (sensorOk) {
        display.printf("dx %+6ld  dy %+6ld\n", (long)fdx, (long)fdy);
        display.printf("SQ %3u %-3s Wh %+6ld\n", s.squal, s.surface ? "ON" : "OFF", (long)s.totalWheel);
        display.printf("Btn %c%c%c%c %s",
                       (s.buttons & MOUSE_LEFT) ? 'L' : '-',
                       (s.buttons & MOUSE_MIDDLE) ? 'M' : '-',
                       (s.buttons & MOUSE_RIGHT) ? 'R' : '-',
                       (s.buttons & BTN_MODE) ? 'D' : '-',
                       mouse.isConnected() ? mouse.peerName() : "-");
      } else {
        display.println("PMW3360: not found");
        display.printf("uptime %lus\n", now / 1000);
        // moving bar proves the screen is refreshing
        int x = (now / 20) % 118;
        display.fillRect(x, 25, 10, 6, SSD1306_WHITE);
      }
    }
    // Only push the frame over I2C (~25 ms) when it differs from what the panel shows.
    // The normal screen doesn't change while the mouse moves, so moving costs no I2C traffic.
    static uint8_t shown[128 * 32 / 8];
    static bool shownValid = false;
    if (!shownValid || memcmp(shown, display.getBuffer(), sizeof(shown)) != 0) {
      display.display();
      memcpy(shown, display.getBuffer(), sizeof(shown));
      shownValid = true;
    }
  }
}

static void oledSelfTest() {
  display.fillScreen(SSD1306_WHITE);   // every pixel on
  display.display();
  delay(600);
  display.clearDisplay();
  display.display();
  delay(200);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(28, 2);
  display.print("exskel");
  display.setTextSize(1);
  display.setCursor(31, 21);
  display.printf("OLED @ 0x%02X", oledAddr);
  display.drawRect(0, 0, 128, 32, SSD1306_WHITE);
  display.display();
  delay(1000);
}

void setup() {
  // 80 MHz (lowest BLE allows) + automatic light sleep whenever all tasks wait. min = max: no
  // clock switching while awake, so SPI/I2C timing never changes. Needs the custom_sdkconfig.
  esp_pm_config_t pm = {.max_freq_mhz = 80, .min_freq_mhz = 80, .light_sleep_enable = true};
  esp_err_t pmErr = esp_pm_configure(&pm);

  // Channel switch (ESP.restart) or deep sleep wake: skip the boot delays so it reconnects fast
  bool quickBoot = esp_reset_reason() == ESP_RST_SW || esp_reset_reason() == ESP_RST_DEEPSLEEP;

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // no host reading USB CDC -> drop output instead of blocking ~100 ms per print
  if (SERIAL_LOG && !quickBoot) delay(1500);  // give USB CDC time to enumerate; does not block without a PC
  LOGF("\n=== exskel boot ===\nesp_pm_configure: %s\n", esp_err_to_name(pmErr));

  // --- Saved settings ---
  prefs.begin("exskel", false);
  bleChannel = prefs.getUChar("ch", 0);
  if (bleChannel >= BLE_CHANNELS) bleChannel = 0;
  cpiIdx = prefs.getUChar("cpi", CPI_DEFAULT_IDX);
  if (cpiIdx >= CPI_COUNT) cpiIdx = CPI_DEFAULT_IDX;
  for (uint8_t i = 0; i < BLE_CHANNELS; i++) {
    char key[3]; nameKey(key, i);
    if (prefs.isKey(key)) prefs.getString(key, chName[i], sizeof(chName[i]));
    addrKey(key, i);
    chHasAddr[i] = prefs.isKey(key) &&
                   prefs.getBytes(key, &chAddr[i], sizeof(chAddr[i])) == sizeof(chAddr[i]);
  }

  // --- OLED ---
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  for (uint8_t a : {0x3C, 0x3D}) {  // the two SSD1306 addresses
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { oledAddr = a; break; }
  }
  if (oledAddr &&display.begin(SSD1306_SWITCHCAPVCC, oledAddr)) {
    oledOk = true;
    display.setTextWrap(false);
    display.setTextColor(SSD1306_WHITE);
    LOGF("OLED OK at 0x%02X\n", oledAddr);
    if (!quickBoot) oledSelfTest();
  } else {
    LOGF("OLED NOT FOUND - check SDA=GPIO8, SCL=GPIO9, VCC=3V3\n");
  }

  // --- PMW3360 ---
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);  // library's own SPI.begin() then no-ops; it drives NCS itself
  sensorOk = sensor.begin(PIN_PMW_CS, CPI_PRESETS[cpiIdx]);
  if (sensorOk) {
    // Library leaves Rest mode off (Config2=0x00, wired). 0x20 enables Rest1/2/3 per
    // datasheet p.23 (wireless design); MOT still fires on motion from any rest state.
    sensor.writeReg(REG_Config2, 0x20);
    LOGF("PMW3360 OK, CPI=%u, Config2=0x%02X\n", sensor.getCPI(), sensor.readReg(REG_Config2));
  } else {
    uint8_t pid = sensor.readReg(REG_Product_ID);
    LOGF("PMW3360 not found (Product_ID=0x%02X, expect 0x42; 0x00/0xFF = wiring)\n", pid);
  }

  // --- BLE ---
  char name[24];
  if (bleChannel) snprintf(name, sizeof(name), "%s %u", BLE_NAME, bleChannel + 1);
  else snprintf(name, sizeof(name), "%s", BLE_NAME);
  mouse.setLockedHost(chHasAddr[bleChannel] ? &chAddr[bleChannel] : nullptr);  // channel lock
  mouse.begin(name, bleChannel);
  LOGF("BLE channel %u advertising as \"%s\"\n", bleChannel + 1, name);

  // --- Tasks ---
  oledQueue = xQueueCreate(1, sizeof(Snapshot));
  loopHandle = xTaskGetCurrentTaskHandle();  // setup() and loop() run in loopTask
  vTaskPrioritySet(nullptr, 2);              // sensor/BLE above the OLED task
  xTaskCreate(oledTask, "oled", 4096, nullptr, 1, nullptr);

  // MOT pull-up also keeps it high when no sensor is fitted
  for (int pin : {PIN_PMW_MOT, PIN_BTN_LEFT, PIN_BTN_RIGHT, PIN_BTN_MIDDLE, PIN_BTN_MODE,
                  PIN_WHEEL_A, PIN_WHEEL_B})
    pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(50);  // let the pull-ups settle, then start the decoder from the real position
  wheelStable = wheelCand = (digitalRead(PIN_WHEEL_A) << 1) | digitalRead(PIN_WHEEL_B);

  // Level interrupts re-armed by each ISR (see rearm); each pin is also a light sleep wake source.
  // Attached as ONLOW: a pin that already reads low fires once and the ISR re-arms it.
  auto wakeOn = [](int pin) {
    gpio_wakeup_enable((gpio_num_t)pin, digitalRead(pin) ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
  };
  for (int pin : {PIN_PMW_MOT, PIN_BTN_LEFT, PIN_BTN_RIGHT, PIN_BTN_MIDDLE, PIN_BTN_MODE}) {
    if (pin == PIN_PMW_MOT && !sensorOk) continue;
    attachInterruptArg(pin, inputIsr, (void*)(intptr_t)pin, ONLOW);
    wakeOn(pin);
  }
  for (int pin : {PIN_WHEEL_A, PIN_WHEEL_B}) {
    attachInterrupt(pin, wheelIsr, ONLOW);
    wakeOn(pin);
  }
  esp_sleep_enable_gpio_wakeup();
}

// Leave the menu. Cancel restores the previewed CPI; confirm saves to NVS.
static void menuClose(Snapshot& s, bool confirm) {
  if (s.menu == MENU_CPI) {
    if (confirm && s.sel != cpiIdx) { cpiIdx = s.sel; prefs.putUChar("cpi", cpiIdx); }
    applyCpi(cpiIdx);
  } else if (s.menu == MENU_BT && confirm && s.sel != bleChannel) {
    prefs.putUChar("ch", s.sel);
    s.switching = true;  // loop() reboots after the OLED shows it
  }
  s.menu = MENU_NONE;
}

void loop() {
  static Snapshot s = {};
  static uint8_t lastHostButtons = 0, rawPrev = 0, prevButtons = 0;
  static uint32_t rawSince = 0, menuSince = 0, rightSince = 0;
  static bool rightArmed = false;  // BT menu: right pressed, release = cancel, hold = clear
  static bool suppress = false;  // after a menu closes: hold host buttons at 0 until all are released

  // Sleep until an interrupt fires. Skip the sleep while motion is still pending
  // (MOT low) or a button change is being debounced. In a menu wake every 100 ms for the timeout.
  // A wheel state still settling (debounce) needs a timed wake: no further edge may come.
  bool btnPending = readButtons() != s.buttons;
  if (digitalRead(PIN_PMW_MOT) == HIGH && !btnPending)
    ulTaskNotifyTake(pdTRUE, wheelSettle() ? pdMS_TO_TICKS(2)
                           : (s.menu != MENU_NONE || mouse.needsPoll()) ? pdMS_TO_TICKS(100)
                           : pdMS_TO_TICKS(SLEEP_AFTER_MS));  // timeout = time to check for deep sleep

  int16_t dx = 0, dy = 0;
  if (sensorOk) {
    PMW3360_DATA d = sensor.readBurst();
    s.squal = d.SQUAL;
    s.surface = d.isOnSurface;
    if (d.isOnSurface && d.isMotion) {
      dx = d.dx;
      dy = d.dy;
      if (SWAP_XY) { int16_t t = dx; dx = dy; dy = t; }
      if (INVERT_X) dx = -dx;
      if (INVERT_Y) dy = -dy;
    }
  }

  // Debounce: accept a button state once it's been stable for 20 ms
  uint8_t raw = readButtons();
  if (raw != rawPrev) { rawPrev = raw; rawSince = millis(); }
  else if (millis() - rawSince >= 20) s.buttons = raw;
  uint8_t pressed = s.buttons & ~prevButtons;
  uint8_t released = prevButtons & ~s.buttons;
  prevButtons = s.buttons;

  // Whole detents since last time; a partial detent stays in wheelRaw for later
  static int32_t wheelTaken = 0;
  wheelSettle();
  int32_t detents = (wheelRaw - wheelTaken) / WHEEL_STEPS_PER_DETENT;
  detents = constrain(detents, -127, 127);
  wheelTaken += detents * WHEEL_STEPS_PER_DETENT;
  if (INVERT_WHEEL) detents = -detents;

  // ---- Menu ----
  uint8_t oldMenu = s.menu, oldSel = s.sel, oldHold = s.clearHold;
  int8_t wheel = 0;
  if (pressed & BTN_MODE) {  // normal -> BT -> CPI -> normal (cancel)
    if (s.menu == MENU_NONE)     { s.menu = MENU_BT;  s.sel = bleChannel; }
    else if (s.menu == MENU_BT)  { s.menu = MENU_CPI; s.sel = cpiIdx; }
    else                         menuClose(s, false);
    rightArmed = false;
  } else if (s.menu != MENU_NONE) {
    if (detents) {  // wheel up = next item
      uint8_t count = s.menu == MENU_BT ? BLE_CHANNELS : CPI_COUNT;
      s.sel = constrain((int32_t)s.sel + detents, 0, count - 1);
      if (s.menu == MENU_CPI && s.sel != oldSel) applyCpi(s.sel);  // live preview
      if (s.sel != oldSel) rightArmed = false;  // scrolling away aborts a clear
    }
    if (s.menu == MENU_BT && (pressed & MOUSE_RIGHT)) { rightArmed = true; rightSince = millis(); }
    uint32_t held = millis() - rightSince;

    if (pressed & MOUSE_LEFT) {
      rightArmed = false;
      menuClose(s, true);
    } else if (s.menu == MENU_BT && rightArmed && (released & MOUSE_RIGHT)) {
      rightArmed = false;  // short press: cancel
      menuClose(s, false);
    } else if (s.menu == MENU_BT && rightArmed && held >= CLEAR_HOLD_MS) {
      rightArmed = false;  // long hold: clear, stay in the menu (release then does nothing)
      clearChannel(s.sel);
    } else if (s.menu != MENU_BT && (pressed & MOUSE_RIGHT)) {
      menuClose(s, false);
    } else if (!rightArmed && millis() - menuSince >= MENU_TIMEOUT_MS) {
      menuClose(s, false);
    }
    // countdown shown after a brief hold so a quick cancel doesn't flash it
    s.clearHold = (s.menu == MENU_BT && rightArmed && held >= 300)
                ? (CLEAR_HOLD_MS - held + 999) / 1000 : 0;
  } else {
    wheel = detents;
  }
  if (s.menu == MENU_NONE) s.clearHold = 0;
  bool menuChanged = s.menu != oldMenu || s.sel != oldSel || detents || pressed || released ||
                     s.clearHold != oldHold;
  if (s.menu != MENU_NONE && menuChanged) menuSince = millis();
  if (oldMenu != MENU_NONE && s.menu == MENU_NONE) suppress = true;

  // Host never sees menu clicks: buttons read 0 inside a menu (entering releases any held button)
  // and stay 0 after it closes until everything is let go.
  if (suppress && !(s.buttons & (MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE))) suppress = false;
  uint8_t hostButtons = (s.menu != MENU_NONE || suppress) ? 0
                      : s.buttons & (MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE);

  // Remember the bonded host's address (for clearing) and name for this channel's menu entry
  ble_addr_t id;
  if (mouse.takePeerId(id) &&
      (!chHasAddr[bleChannel] || memcmp(&id, &chAddr[bleChannel], sizeof(id)) != 0)) {
    chAddr[bleChannel] = id;
    chHasAddr[bleChannel] = true;
    char key[3]; addrKey(key, bleChannel);
    prefs.putBytes(key, &id, sizeof(id));
    mouse.setLockedHost(&chAddr[bleChannel]);  // first host on this channel: lock it
  }
  if (mouse.needsPoll()) mouse.poll();
  const char* peer = mouse.peerName();
  if (mouse.peerNameReady() && strcmp(peer, chName[bleChannel]) != 0) {
    strlcpy(chName[bleChannel], peer, sizeof(chName[bleChannel]));
    char key[3]; nameKey(key, bleChannel);
    prefs.putString(key, chName[bleChannel]);
  }

  if (dx || dy || wheel || hostButtons != lastHostButtons) {
    if (mouse.isConnected()) {
      mouse.send(hostButtons, dx, dy, wheel);
      s.reports++;
    }
    lastHostButtons = hostButtons;
    s.totalX += dx;
    s.totalY += dy;
    s.totalWheel += wheel;
  }
  if (dx || dy || wheel || menuChanged || s.switching)
    xQueueOverwrite(oledQueue, &s);  // new data only -> OLED task redraws

  // No host and idle too long -> deep sleep. While connected, light sleep saves power without
  // dropping the link (Windows often fails to reconnect after a wake). Wake = reboot.
  // Only GPIO0-5 can wake the C3 from deep sleep: MOT(3) and left(1). Wheel A(0) may rest low,
  // mode(2) is a strap pin.
  static uint32_t lastInput = 0;
  if (dx || dy || detents || raw) mouse.wakeAdv();  // touched while no host: fast advertising again
  if (dx || dy || detents || raw || s.menu != MENU_NONE || mouse.isConnected()) lastInput = millis();
  else if (!DEBUG_MODE && sensorOk && millis() - lastInput >= SLEEP_AFTER_MS) {
    sensor.readBurst();  // clear pending motion so MOT is high
    esp_deep_sleep_enable_gpio_wakeup((1ULL << PIN_PMW_MOT) | (1ULL << PIN_BTN_LEFT),
                                      ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
  }

  if (s.switching) {  // channel confirmed: let the OLED show it, drop the link, reboot as the new identity
    vTaskDelay(pdMS_TO_TICKS(600));
    mouse.disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP.restart();
  }

  // Pace reads to the BLE connection interval: one read (the sensor accumulates motion in between)
  // = one report per connection event, so every event carries the same slice of time.
  // Sleep most of the gap, then busy-wait the last bit for a precise slot (tick is 1 ms).
  // 64-bit timer: 32-bit micros() wraps every 71.6 min, and after >35.8 min idle the signed
  // compare flipped, so loop() slept for up to half an hour and ignored all input.
  static int64_t nextUs = 0;
  int64_t periodUs = mouse.connInterval() ? mouse.connInterval() * 1250 : MOUSE_PERIOD_MS * 1000;
  int64_t nowUs = esp_timer_get_time();
  if (nowUs - nextUs > periodUs) nextUs = nowUs;  // was idle: restart cadence
  nextUs += periodUs;
  int64_t leftUs = nextUs - esp_timer_get_time();
  if (leftUs > 2000) vTaskDelay(pdMS_TO_TICKS((leftUs - 1000) / 1000));
  while (esp_timer_get_time() < nextUs) {}
}
