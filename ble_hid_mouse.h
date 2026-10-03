// Minimal BLE HID mouse on NimBLE-Arduino 2.x
// Report (ID 1): [buttons(3 bits + 5 pad)] [X int16 LE] [Y int16 LE] [wheel int8]
#pragma once
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

#define MOUSE_LEFT   0x01
#define MOUSE_RIGHT  0x02
#define MOUSE_MIDDLE 0x04

static const uint8_t kMouseReportMap[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x01,        //   Report ID (1)
  0x09, 0x01,        //   Usage (Pointer)
  0xA1, 0x00,        //   Collection (Physical)
  0x05, 0x09,        //     Usage Page (Button)
  0x19, 0x01,        //     Usage Minimum (1)
  0x29, 0x03,        //     Usage Maximum (3)
  0x15, 0x00,        //     Logical Minimum (0)
  0x25, 0x01,        //     Logical Maximum (1)
  0x95, 0x03,        //     Report Count (3)
  0x75, 0x01,        //     Report Size (1)
  0x81, 0x02,        //     Input (Data, Var, Abs)
  0x95, 0x01,        //     Report Count (1)
  0x75, 0x05,        //     Report Size (5)
  0x81, 0x03,        //     Input (Const) - padding
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x30,        //     Usage (X)
  0x09, 0x31,        //     Usage (Y)
  0x16, 0x01, 0x80,  //     Logical Minimum (-32767)
  0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
  0x75, 0x10,        //     Report Size (16)
  0x95, 0x02,        //     Report Count (2)
  0x81, 0x06,        //     Input (Data, Var, Rel)
  0x09, 0x38,        //     Usage (Wheel)
  0x15, 0x81,        //     Logical Minimum (-127)
  0x25, 0x7F,        //     Logical Maximum (127)
  0x75, 0x08,        //     Report Size (8)
  0x95, 0x01,        //     Report Count (1)
  0x81, 0x06,        //     Input (Data, Var, Rel)
  0xC0,              //   End Collection
  0xC0               // End Collection
};

class BleHidMouse : public NimBLEServerCallbacks {
 public:
  void begin(const char* name) {
    NimBLEDevice::init(name);
    NimBLEDevice::setSecurityAuth(true, false, true);  // bond, no MITM, secure connections

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(this);

    hid_ = new NimBLEHIDDevice(server);
    input_ = hid_->getInputReport(1);
    hid_->setManufacturer("exskel");
    hid_->setPnp(0x02, 0xe502, 0xa111, 0x0210);
    hid_->setHidInfo(0x00, 0x01);
    hid_->setReportMap((uint8_t*)kMouseReportMap, sizeof(kMouseReportMap));
    hid_->startServices();
    hid_->setBatteryLevel(100);

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setAppearance(0x03C2);  // HID mouse
    adv->addServiceUUID(hid_->getHidService()->getUUID());
    adv->setName(name);
    adv->enableScanResponse(true);
    adv->start();
  }

  bool isConnected() const { return connected_; }

  // wheel > 0 = scroll up
  void send(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel) {
    if (!connected_) return;
    uint8_t r[6] = {buttons,
                    (uint8_t)(dx & 0xFF), (uint8_t)((uint16_t)dx >> 8),
                    (uint8_t)(dy & 0xFF), (uint8_t)((uint16_t)dy >> 8),
                    (uint8_t)wheel};
    input_->setValue(r, sizeof(r));
    input_->notify();
  }

  // NimBLEServerCallbacks
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    connected_ = true;
    // 7.5–15 ms connection interval for smooth cursor movement
    server->updateConnParams(info.getConnHandle(), 6, 12, 0, 200);
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    connected_ = false;
    NimBLEDevice::startAdvertising();
  }

 private:
  NimBLEHIDDevice* hid_ = nullptr;
  NimBLECharacteristic* input_ = nullptr;
  volatile bool connected_ = false;
};
