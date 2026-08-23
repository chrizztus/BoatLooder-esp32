// Minimal OTA-receiver-only sketch -- the one-time bootstrap for a fresh
// ESP32 that can't build the real boatlooder-esp32 PlatformIO project
// (missing TMCStepper/AccelStepper/MAVLink toolchain). Flash this once
// over USB with Arduino IDE, then the app installs the real firmware
// entirely over BLE -- see README.md in this folder for the one setting
// that has to match exactly (the partition scheme).
//
// Deliberately self-contained and minimal: only built-in ESP32 Arduino
// libraries (BLE, Update.h), nothing from Library Manager. Speaks the
// *same* 3-characteristic OTA protocol as the real firmware's
// BluetoothHandler.cpp/OtaUpdate.h -- same UUIDs, same frame layout, so
// the app's OtaController doesn't need to know or care which one it's
// talking to. Some duplication against that real implementation is
// deliberate (a friend who can barely get Arduino IDE working needs one
// file they can just open and click Upload, not a shared library to
// install first) -- if the protocol ever changes, update both.

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Update.h>

// Same device name and service the real firmware advertises under
// (BluetoothHandler.h/VesselConfig.h's APP_FILTER_PREFIX/DEFAULT_VESSEL_NAME)
// -- the app finds a vessel by name prefix regardless of which firmware
// (bootstrap or real) is currently running. This sketch has no rename
// persistence of its own (deliberately minimal), so it always advertises
// the default name -- a rename only takes effect once the real firmware
// is installed.
#define DEVICE_NAME "boatlooder@boatlooder"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

// Identical to BluetoothHandler.cpp's OTA_*_CHARACTERISTIC_UUID defines.
#define OTA_VERSION_CHARACTERISTIC_UUID "c91a5c87-d8cc-44ba-b8f7-5ddde24493bf"
#define OTA_CONTROL_CHARACTERISTIC_UUID "37ede416-4d9e-48c9-afdb-587f726b4658"
#define OTA_DATA_CHARACTERISTIC_UUID    "fe5f1b67-8505-4ab6-b40f-246abea4c93d"

// Error codes -- must match OtaUpdate.h's OtaError enum and
// boatlooder-app's OtaController exactly.
enum OtaErrorCode : uint8_t {
  OTA_ERR_ALREADY_IN_PROGRESS = 0,
  OTA_ERR_INVALID_SIZE = 1,
  OTA_ERR_BEGIN_FAILED = 2,
  OTA_ERR_WRITE_FAILED = 3,
  OTA_ERR_END_FAILED = 4,
  OTA_ERR_NOT_STARTED = 5,
};

BLECharacteristic *otaControlChar = nullptr;
bool otaInProgress = false;

void notifyOtaControl(const uint8_t *data, size_t length) {
  if (otaControlChar == nullptr) return;
  otaControlChar->setValue((uint8_t *)data, length);
  otaControlChar->notify();
}

void otaBegin(uint32_t size) {
  if (otaInProgress) {
    uint8_t frame[2] = {0x03, OTA_ERR_ALREADY_IN_PROGRESS};
    notifyOtaControl(frame, sizeof(frame));
    return;
  }
  if (size == 0) {
    uint8_t frame[2] = {0x03, OTA_ERR_INVALID_SIZE};
    notifyOtaControl(frame, sizeof(frame));
    return;
  }
  if (!Update.begin(size)) {
    uint8_t frame[2] = {0x03, OTA_ERR_BEGIN_FAILED};
    notifyOtaControl(frame, sizeof(frame));
    return;
  }
  otaInProgress = true;
  uint8_t frame[1] = {0x01};
  notifyOtaControl(frame, sizeof(frame));
}

void otaWriteChunk(const uint8_t *data, size_t length) {
  if (!otaInProgress) {
    uint8_t frame[2] = {0x03, OTA_ERR_NOT_STARTED};
    notifyOtaControl(frame, sizeof(frame));
    return;
  }
  size_t written = Update.write(const_cast<uint8_t *>(data), length);
  if (written != length) {
    Update.abort();
    otaInProgress = false;
    uint8_t frame[2] = {0x03, OTA_ERR_WRITE_FAILED};
    notifyOtaControl(frame, sizeof(frame));
  }
}

void rebootTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(500)); // let the OK notify actually clear the BLE link
  ESP.restart();
}

void otaEnd() {
  if (!otaInProgress) {
    uint8_t frame[2] = {0x03, OTA_ERR_NOT_STARTED};
    notifyOtaControl(frame, sizeof(frame));
    return;
  }
  otaInProgress = false;
  if (!Update.end(true) || !Update.isFinished()) {
    uint8_t frame[2] = {0x03, OTA_ERR_END_FAILED};
    notifyOtaControl(frame, sizeof(frame));
    return;
  }
  uint8_t frame[1] = {0x02};
  notifyOtaControl(frame, sizeof(frame));
  xTaskCreate(rebootTask, "reboot", 2048, nullptr, 1, nullptr);
}

void otaAbort() {
  if (!otaInProgress) return;
  Update.abort();
  otaInProgress = false;
}

class OtaControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    const uint8_t *data = c->getData();
    size_t length = c->getLength();
    if (length < 1) return;
    switch (data[0]) {
      case 0x01: { // BEGIN: size (u32 LE), verLen (u8), verBytes[verLen] (unused here)
        if (length < 5) return;
        uint32_t size = (uint32_t)data[1] | ((uint32_t)data[2] << 8) |
                         ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
        otaBegin(size);
        break;
      }
      case 0x02: // END
        otaEnd();
        break;
      case 0x03: // ABORT
        otaAbort();
        break;
      default:
        break;
    }
  }
};

class OtaDataCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    otaWriteChunk(c->getData(), c->getLength());
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onDisconnect(BLEServer *server) override {
    // Same reasoning as the real firmware's onBluetoothDisconnect(): a
    // dropped link mid-transfer must not leave a half-written image
    // looking recoverable later.
    otaAbort();
    server->getAdvertising()->start();
  }
};

void setup() {
  Serial.begin(115200);

  BLEDevice::init(DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  // Explicit handle count, not the default 15 -- see BluetoothHandler.cpp's
  // own comment on this exact gotcha (a characteristic added past the
  // table's limit simply never appears over BLE, no error). Only 3
  // characteristics here (one with a descriptor) need far fewer than 15,
  // but matching the real firmware's explicit style avoids re-discovering
  // the same bug if this file ever grows.
  BLEService *service = server->createService(BLEUUID(SERVICE_UUID), 20);

  BLECharacteristic *versionChar = service->createCharacteristic(
      OTA_VERSION_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ);
  versionChar->setValue("bootstrap");

  otaControlChar = service->createCharacteristic(
      OTA_CONTROL_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  otaControlChar->addDescriptor(new BLE2902());
  otaControlChar->setCallbacks(new OtaControlCallbacks());

  BLECharacteristic *dataChar = service->createCharacteristic(
      OTA_DATA_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
  dataChar->setCallbacks(new OtaDataCallbacks());

  service->start();
  server->getAdvertising()->start();

  Serial.println("OTA bootstrap ready -- advertising as " DEVICE_NAME);
}

void loop() {
  // Nothing to do -- everything happens in BLE callbacks.
}
