// Minimal BLE HID mouse on NimBLE-Arduino 2.x
// Report (ID 1): [buttons(3 bits + 5 pad)] [X int16 LE] [Y int16 LE] [wheel int8]
//
// Channels: each channel is a separate BLE identity (own address), so each host bonds
// to its own channel and only reconnects when that channel is active.
//   channel 0 = chip's public address (keeps pairings made before channels existed)
//   channel 1.. = static random address derived from the chip MAC
#pragma once
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <esp_mac.h>

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
  void begin(const char* name, uint8_t channel = 0) {
    NimBLEDevice::init(name);
    if (channel) {
      uint8_t a[6];
      esp_read_mac(a, ESP_MAC_BT);
      a[0] ^= channel;   // different address per channel
      a[5] |= 0xC0;      // static random address: top two bits of the MSB (byte 5, little-endian) set
      ble_hs_id_set_rnd(a);
      NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
    }
    NimBLEDevice::setSecurityAuth(true, false, true);  // bond, no MITM, secure connections

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(this);

    auto* hid = new NimBLEHIDDevice(server);
    input_ = hid->getInputReport(1);
    hid->setManufacturer("exskel");
    hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->setHidInfo(0x00, 0x01);
    hid->setReportMap((uint8_t*)kMouseReportMap, sizeof(kMouseReportMap));
    hid->startServices();
    hid->setBatteryLevel(100);

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setAppearance(0x03C2);  // HID mouse
    adv->addServiceUUID(hid->getHidService()->getUUID());
    adv->setName(name);
    adv->enableScanResponse(true);
    adv->start();
  }

  // Channel lock: only this host (identity address) may use the channel; nullptr = open, any host
  // may pair. Another host is dropped once its identity is known, and its new bond deleted.
  void setLockedHost(const ble_addr_t* id) {
    if (id) lockedId_ = *id;
    locked_ = id != nullptr;
  }

  // Connected to a host that may use this channel (reports are only sent then)
  bool isConnected() const { return connected_ && authorized_; }
  // Current connection interval in 1.25 ms units (0 = not known)
  uint16_t connInterval() const { return connected_ ? connItv_ : 0; }

  void disconnect() {
    if (connected_) NimBLEDevice::getServer()->disconnect(connHandle_);
  }

  // wheel > 0 = scroll up
  void send(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel) {
    if (!isConnected()) return;
    uint8_t r[6] = {buttons,
                    (uint8_t)(dx & 0xFF), (uint8_t)((uint16_t)dx >> 8),
                    (uint8_t)(dy & 0xFF), (uint8_t)((uint16_t)dy >> 8),
                    (uint8_t)wheel};
    input_->setValue(r, sizeof(r));
    input_->notify();
  }

  // Host's name, read from its GAP Device Name (0x2A00); shows its address until the reply arrives.
  // One async ATT "read by type" request: never blocks, the answer lands in onNameRead.
  const char* peerName() {
    if (namePending_ && millis() - connectedAt_ >= 1000) {  // let pairing/encryption settle first
      namePending_ = false;
      static const ble_uuid16_t kDeviceName = BLE_UUID16_INIT(0x2A00);
      ble_gattc_read_by_uuid(connHandle_, 1, 0xFFFF, &kDeviceName.u, onNameRead, this);
    }
    return peerName_;
  }
  // real name, not the address; only on a bonded link so a host with stale keys isn't remembered
  bool peerNameReady() const { return connected_ && nameReady_ && bonded_; }

  // Bonded host's identity address, handed out once per (re)connection
  bool takePeerId(ble_addr_t& out) {
    if (!idReady_) return false;
    out = peerId_;
    idReady_ = false;
    return true;
  }

  // Forget a host's bond (keys); drops the link if that host is connected
  static void unbond(const ble_addr_t& id) { NimBLEDevice::deleteBond(NimBLEAddress(id)); }

  // Channel lock housekeeping, call from loop() while needsPoll():
  // delete a rejected host's bond once it's gone; drop a link that never proved it's the owner.
  bool needsPoll() const { return rejectPending_ || (connected_ && !authorized_); }
  void poll() {
    if (rejectPending_ && !connected_) {
      rejectPending_ = false;
      unbond(rejectId_);
      LOGF("BLE: rejected host's bond deleted\n");
    }
    if (connected_ && !authorized_ && millis() - connectedAt_ >= AUTH_TIMEOUT_MS) {
      LOGF("BLE: host did not authenticate as the channel owner, dropping\n");
      NimBLEDevice::getServer()->disconnect(connHandle_);
    }
  }

  // NimBLEServerCallbacks
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    connHandle_ = info.getConnHandle();
    snprintf(peerName_, sizeof(peerName_), "%s", info.getAddress().toString().c_str());
    connectedAt_ = millis();
    nameReady_ = false;
    namePending_ = true;
    bonded_ = false;
    authorized_ = !locked_;  // locked channel: wait until encryption reveals who this is
    connected_ = true;
    logConnParams("connect", info);
    // Ask the host to encrypt now (stored keys, or pairing if new): that reveals its identity
    // address, which the channel lock checks in onAuthenticationComplete.
    NimBLEDevice::startSecurity(connHandle_);
    // Ask for 7.5 ms only: given 7.5–15 Windows picked 15 ms (measured 2026-11-12)
    server->updateConnParams(connHandle_, 6, 6, 0, 200);
  }
  // Host accepted (or chose) new connection parameters
  void onConnParamsUpdate(NimBLEConnInfo& info) override { logConnParams("update", info); }
  void onAuthenticationComplete(NimBLEConnInfo& info) override {
    if (!info.isBonded()) return;
    ble_addr_t id = *info.getIdAddress().getBase();
    if (locked_ && memcmp(&id, &lockedId_, sizeof(id)) != 0) {  // not this channel's host
      rejectId_ = id;
      rejectPending_ = true;  // its fresh bond is deleted in poll() after the link is gone
      LOGF("BLE: %s is not this channel's host, dropping\n", info.getIdAddress().toString().c_str());
      NimBLEDevice::getServer()->disconnect(info.getConnHandle());
      return;
    }
    peerId_ = id;
    bonded_ = true;
    authorized_ = true;
    idReady_ = true;
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    connected_ = false;
    authorized_ = false;
    bonded_ = false;
    namePending_ = false;
    nameReady_ = false;
    NimBLEDevice::startAdvertising();
  }

 private:
  // interval unit 1.25 ms, timeout unit 10 ms
  void logConnParams(const char* what, NimBLEConnInfo& info) {
    uint16_t itv = info.getConnInterval();
    connItv_ = itv;
    LOGF("BLE conn params (%s): interval %u.%02u ms, latency %u, timeout %u ms\n", what,
                  itv * 125 / 100, itv * 125 % 100, info.getConnLatency(), info.getConnTimeout() * 10);
  }

  static int onNameRead(uint16_t, const ble_gatt_error* err, ble_gatt_attr* attr, void* arg) {
    if (err->status == 0 && attr) {
      auto* self = static_cast<BleHidMouse*>(arg);
      char buf[sizeof(peerName_)] = "";
      uint16_t n = OS_MBUF_PKTLEN(attr->om);
      if (n >= sizeof(buf)) n = sizeof(buf) - 1;
      os_mbuf_copydata(attr->om, 0, n, buf);
      if (n) { memcpy(self->peerName_, buf, sizeof(buf)); self->nameReady_ = true; }
    }
    return 0;
  }

  NimBLECharacteristic* input_ = nullptr;
  static constexpr uint32_t AUTH_TIMEOUT_MS = 5000;  // locked channel: owner must encrypt by then
  volatile bool connected_ = false;
  volatile bool authorized_ = false;
  volatile bool locked_ = false;
  volatile bool rejectPending_ = false;
  ble_addr_t lockedId_ = {};
  ble_addr_t rejectId_ = {};
  volatile bool namePending_ = false;
  volatile bool nameReady_ = false;
  volatile bool bonded_ = false;
  volatile bool idReady_ = false;
  ble_addr_t peerId_ = {};
  volatile uint16_t connHandle_ = 0;
  volatile uint32_t connectedAt_ = 0;
  volatile uint16_t connItv_ = 0;
  char peerName_[32] = "";
};
