// exskel - Nologo ESP32C3 Super Mini + 0.96" SSD1315 OLED (I2C) + PMW3360 -> BLE mouse
//
// Wiring (board default pins):
//   OLED    VCC-3V3  GND-GND  SDA-GPIO8  SCL-GPIO9
//   PMW3360 VDD-3V3 (module has 2.0 V LDO)  VDDIO-3V3  GND-GND  SCLK-GPIO4  MISO-GPIO5
//           MOSI-GPIO6  NCS-GPIO7  MOT-GPIO3  NRESET open or 3V3 (internal pull-up)
//   Buttons left: GPIO1 -> button -> GND   right: GPIO10 -> button -> GND
//
// The PMW3360 is detected at boot. Without it the OLED runs a test screen.
//
// Tasks:
//   loop()   (prio 2) sensor + BLE. Sleeps until MOT or a button fires, reads, sends the
//                     report, and only when there is new data hands a snapshot to the OLED task.
//   oledTask (prio 1) redraws the OLED from the latest snapshot. Lower priority, so the
//                     ~25 ms I2C frame transfer never delays the cursor.

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PMW3360.h>
#include "ble_hid_mouse.h"

// ---------------- Config ----------------
constexpr int PIN_I2C_SDA  = 8;
constexpr int PIN_I2C_SCL  = 9;
constexpr int PIN_SPI_SCK  = 4;
constexpr int PIN_SPI_MISO = 5;
constexpr int PIN_SPI_MOSI = 6;
constexpr int PIN_PMW_CS   = 7;
constexpr int PIN_PMW_MOT  = 3;      // MOTION, active low: sensor pulls it low when it has moved

constexpr unsigned PMW_CPI = 1600;   // 100..12000, step 100
constexpr bool INVERT_X = false;
constexpr bool INVERT_Y = false;
constexpr bool SWAP_XY  = false;

constexpr int PIN_BTN_LEFT  = 1;     // pin -> button -> GND (internal pull-up)
constexpr int PIN_BTN_RIGHT = 10;

const char* BLE_NAME = "exskel Mouse";
constexpr uint32_t MOUSE_PERIOD_MS = 8;    // read rate while moving
constexpr uint32_t OLED_TIMEOUT_MS = 10000; // OLED sleeps after this long without motion/buttons
// ----------------------------------------

Adafruit_SSD1306 display(128, 64, &Wire, -1);
PMW3360 sensor;
BleHidMouse mouse;

bool oledOk = false;
uint8_t oledAddr = 0;
bool sensorOk = false;

// loop() -> oledTask hand-off. Length-1 queue + xQueueOverwrite = "latest value" mailbox.
// Totals are running sums, so a skipped snapshot loses no movement.
struct Snapshot {
  int32_t  totalX, totalY;
  uint32_t reports;
  uint8_t  squal;
  bool     surface;
  uint8_t  buttons;
};
QueueHandle_t oledQueue;

TaskHandle_t loopHandle;  // loopTask, woken by MOT / button interrupts

// MOT falling edge / button edge -> wake loop()
void IRAM_ATTR wakeLoop() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(loopHandle, &woken);
  portYIELD_FROM_ISR(woken);
}

static uint8_t readButtons() {
  uint8_t b = 0;
  if (digitalRead(PIN_BTN_LEFT) == LOW)  b |= MOUSE_LEFT;
  if (digitalRead(PIN_BTN_RIGHT) == LOW) b |= MOUSE_RIGHT;
  return b;
}

void oledTask(void*) {
  Snapshot s = {};
  int32_t frameX = 0, frameY = 0, secX = 0, secY = 0;
  uint32_t lastRate = 0, lastReports = 0, rate = 0;
  bool moving = false;
  bool oledOn = true;
  uint32_t lastActivity = millis();

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
    if (fresh) lastActivity = now;

    int32_t fdx = s.totalX - frameX, fdy = s.totalY - frameY;
    frameX = s.totalX; frameY = s.totalY;
    moving = fdx || fdy;

    if (now - lastRate >= 1000) {
      rate = s.reports - lastReports;
      lastReports = s.reports;
      lastRate = now;
      if (s.totalX != secX || s.totalY != secY)  // Serial only while moving
        Serial.printf("BLE:%s dx/s:%ld dy/s:%ld SQUAL:%u surf:%d rep/s:%u\n",
                      mouse.isConnected() ? "conn" : "adv",
                      (long)(s.totalX - secX), (long)(s.totalY - secY), s.squal, s.surface, rate);
      secX = s.totalX; secY = s.totalY;
    }

    if (!oledOk) continue;

    // Idle -> panel sleep (0xAE, ~10 uA, RAM kept). Any new data wakes it (0xAF).
    if (sensorOk && now - lastActivity >= OLED_TIMEOUT_MS) {
      if (oledOn) { display.ssd1306_command(SSD1306_DISPLAYOFF); oledOn = false; }
      continue;
    }
    if (!oledOn) { display.ssd1306_command(SSD1306_DISPLAYON); oledOn = true; }

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.printf("BLE: %s\n", mouse.isConnected() ? "CONNECTED" : "advertising");
    display.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    display.setCursor(0, 12);
    if (sensorOk) {
      display.printf("PMW3360 OK  CPI %u\n", PMW_CPI);
      display.printf("dx %+6ld  dy %+6ld\n", (long)fdx, (long)fdy);
      display.printf("SQUAL %3u  Surf %s\n", s.squal, s.surface ? "Y" : "N");
      display.printf("Btn %c%c   rep/s %u\n",
                     (s.buttons & MOUSE_LEFT) ? 'L' : '-',
                     (s.buttons & MOUSE_RIGHT) ? 'R' : '-', rate);
    } else {
      display.println("PMW3360: not found");
      display.println("(OLED test mode)");
      display.printf("uptime %lus\n", now / 1000);
      // moving bar proves the screen is refreshing
      int x = (now / 20) % 118;
      display.fillRect(x, 50, 10, 6, SSD1306_WHITE);
    }
    display.display();
  }
}

static void i2cScan() {
  Serial.println("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found 0x%02X\n", a);
      if ((a == 0x3C || a == 0x3D) && !oledAddr) oledAddr = a;
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
  display.setCursor(22, 10);
  display.print("exskel");
  display.setTextSize(1);
  display.setCursor(22, 36);
  display.printf("OLED @ 0x%02X", oledAddr);
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.display();
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // no host reading USB CDC -> drop output instead of blocking ~100 ms per print
  delay(1500);  // give USB CDC time to enumerate; does not block without a PC
  Serial.println("\n=== exskel boot ===");

  // --- OLED ---
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  i2cScan();
  if (oledAddr && display.begin(SSD1306_SWITCHCAPVCC, oledAddr)) {
    oledOk = true;
    Serial.printf("OLED OK at 0x%02X\n", oledAddr);
    oledSelfTest();
  } else {
    Serial.println("OLED NOT FOUND - check SDA=GPIO8, SCL=GPIO9, VCC=3V3");
  }

  // --- PMW3360 ---
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);  // library's own SPI.begin() then no-ops; it drives NCS itself
  sensorOk = sensor.begin(PIN_PMW_CS, PMW_CPI);
  if (sensorOk) {
    // Library leaves Rest mode off (Config2=0x00, wired). 0x20 enables Rest1/2/3 per
    // datasheet p.23 (wireless design); MOT still fires on motion from any rest state.
    sensor.writeReg(REG_Config2, 0x20);
    Serial.printf("PMW3360 OK, CPI=%u, Config2=0x%02X\n", sensor.getCPI(), sensor.readReg(REG_Config2));
  } else {
    uint8_t pid = sensor.readReg(REG_Product_ID);
    Serial.printf("PMW3360 not found (Product_ID=0x%02X, expect 0x42; 0x00/0xFF = wiring)\n", pid);
  }

  // --- BLE ---
  mouse.begin(BLE_NAME);
  Serial.printf("BLE advertising as \"%s\"\n", BLE_NAME);

  // --- Tasks ---
  oledQueue = xQueueCreate(1, sizeof(Snapshot));
  loopHandle = xTaskGetCurrentTaskHandle();  // setup() and loop() run in loopTask
  vTaskPrioritySet(nullptr, 2);              // sensor/BLE above the OLED task
  xTaskCreate(oledTask, "oled", 4096, nullptr, 1, nullptr);

  pinMode(PIN_PMW_MOT, INPUT_PULLUP);  // also keeps MOT high when no sensor is fitted
  if (sensorOk) attachInterrupt(digitalPinToInterrupt(PIN_PMW_MOT), wakeLoop, FALLING);
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_LEFT), wakeLoop, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_RIGHT), wakeLoop, CHANGE);
}

void loop() {
  static Snapshot s = {};
  static uint8_t lastButtons = 0, rawPrev = 0;
  static uint32_t rawSince = 0;

  // Sleep until MOT or a button edge fires. Skip the sleep while motion is still pending
  // (MOT low) or a button change is being debounced.
  bool btnPending = readButtons() != s.buttons;
  if (digitalRead(PIN_PMW_MOT) == HIGH && !btnPending)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

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

  if (dx || dy || s.buttons != lastButtons) {
    if (mouse.isConnected()) {
      mouse.send(s.buttons, dx, dy);
      s.reports++;
    }
    lastButtons = s.buttons;
    s.totalX += dx;
    s.totalY += dy;
    xQueueOverwrite(oledQueue, &s);  // new data only -> OLED task redraws
  }

  vTaskDelay(pdMS_TO_TICKS(MOUSE_PERIOD_MS));
}
