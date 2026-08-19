#include "BluetoothHandler.h"
#include "Logger.h"
#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TELEMETRY_CHARACTERISTIC_UUID "430f885a-4c7b-40c6-bdfc-280a526fd118"
#define SETTINGS_CHARACTERISTIC_UUID  "5cf3acbf-4809-453a-93ab-4359429056e3"

// MTU we ask the peer for. The negotiated value is the min of both sides' asks,
// so never assume this landed -- usableChunkSize() reads back what we actually got.
#define REQUESTED_MTU 247
// Fallback usable payload when the negotiated MTU can't be read (not connected yet,
// or the peer never ran an MTU exchange and we're still on the 23 byte default).
#define FALLBACK_CHUNK_SIZE 180
// Pause between back-to-back notify()/indicate() calls on the same characteristic.
// The ESP32 BLE stack can drop a packet queued while the previous one is still in
// flight. This value is a conservative guess, NOT a measurement -- it has never been
// checked against real hardware. Tune it once the link can actually be observed.
#define RELAY_CHUNK_PACING_MS 3

class BoatLuderCallbacks: public BLECharacteristicCallbacks {
    BluetoothHandler* handler;
public:
    BoatLuderCallbacks(BluetoothHandler* handler) : handler(handler) {}

    void onWrite(BLECharacteristic *pCharacteristic) {
      // getData()/getLength(), not getValue().c_str(): the control frame is
      // binary and a std::string rebuilt from c_str() stops at the first NUL
      // byte, so any frame carrying a zero (e.g. a pulse value like 1280 ->
      // 0x05 0x00) was silently truncated and then dropped by the length == 8
      // check in onBluetoothWrite().
      const uint8_t* data = pCharacteristic->getData();
      size_t length = pCharacteristic->getLength();
      if (length > 0 && handler->getOnWriteCallback()) {
          handler->getOnWriteCallback()(data, length);
      }
    }
};

// Mirrors BoatLuderCallbacks, but routes writes to the settings (MAVLink param)
// callback instead of the RC control one.
class SettingsCallbacks: public BLECharacteristicCallbacks {
    BluetoothHandler* handler;
public:
    SettingsCallbacks(BluetoothHandler* handler) : handler(handler) {}

    void onWrite(BLECharacteristic *pCharacteristic) {
      // Same NUL-truncation trap as the control characteristic, but fatal
      // here: a MAVLink v2 frame carries NUL bytes in its own header (msgid 20
      // encodes as 0x14 0x00 0x00), so every param request was cut off
      // mid-header and never reassembled into a complete frame.
      const uint8_t* data = pCharacteristic->getData();
      size_t length = pCharacteristic->getLength();
      if (length > 0 && handler->getOnSettingsWriteCallback()) {
          handler->getOnSettingsWriteCallback()(data, length);
      }
    }
};

class ServerCallbacks: public BLEServerCallbacks {
    BluetoothHandler* handler;
public:
    ServerCallbacks(BluetoothHandler* handler) : handler(handler) {}

    void onConnect(BLEServer* pServer) override {
        if (handler->getOnConnectCallback()) {
            handler->setConnectionState(true);
            handler->getOnConnectCallback()();
        }
    }

    void onDisconnect(BLEServer* pServer) override {
        if (handler->getOnDisconnectCallback()) {
            handler->setConnectionState(false);
            handler->getOnDisconnectCallback()();
        }
        pServer->getAdvertising()->start();  // Restart advertising
    }
};

BluetoothHandler::BluetoothHandler() : _telemetryChar(nullptr), _settingsChar(nullptr) {}

void BluetoothHandler::init() {
    LOG_INFO("BT HANLDER INIT :: START");
    BLEDevice::init(DEVICE_NAME);
    // Must come *after* init(): this library version rejects setMTU() before
    // the BLE stack is up ("BLE is not initialized"), which silently left the
    // link at the 23-byte default MTU. Verified on hardware -- the spec's
    // suggested before-init ordering does not work here.
    BLEDevice::setMTU(REQUESTED_MTU);
    BLEServer *pServer = BLEDevice::createServer();

    pServer->setCallbacks(new ServerCallbacks(this));

    BLEService *pService = pServer->createService(SERVICE_UUID);

    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );

    pCharacteristic->setCallbacks(new BoatLuderCallbacks(this));

    pCharacteristic->setValue("BOAT_CTRLS");

    // Telemetry: MAVLink frames relayed out of the UART stream (notify only).
    _telemetryChar = pService->createCharacteristic(
                       TELEMETRY_CHARACTERISTIC_UUID,
                       BLECharacteristic::PROPERTY_NOTIFY
                     );
    _telemetryChar->addDescriptor(new BLE2902());

    // Settings: app writes param requests in, PARAM_VALUE acks go back out.
    // INDICATE (never NOTIFY) -- the ack design relies on the link-layer
    // confirmation, and clients pick notify over indicate when both are offered.
    _settingsChar = pService->createCharacteristic(
                      SETTINGS_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
    _settingsChar->addDescriptor(new BLE2902());
    _settingsChar->setCallbacks(new SettingsCallbacks(this));

    pService->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    LOG_INFO("BT HANDLER INIT :: DONE");

    this->_isConnected = false;
  }

// Setter implementations
void BluetoothHandler::setOnWriteCallback(OnWriteCallback callback) {
    LOG_DEBUG("BT HANLDER WRITE CB");
    this->_onWriteCallback = callback;
}

void BluetoothHandler::setOnSettingsWriteCallback(OnSettingsWriteCallback callback) {
    LOG_DEBUG("BT HANLDER SETTINGS WRITE CB");
    this->_onSettingsWriteCallback = callback;
}

void BluetoothHandler::setOnConnectCallback(OnConnectCallback callback) {
    LOG_DEBUG("BT HANLDER CONNECT CB");
    this->_onConnectCallback = callback;
}

void BluetoothHandler::setOnDisconnectCallback(OnDisconnectCallback callback) {
    LOG_DEBUG("BT HANLDER DISCONNECT CB");
    this->_onDisconnectCallback = callback;
}

// Getter implementations
OnWriteCallback BluetoothHandler::getOnWriteCallback() const {
    return this->_onWriteCallback;
}

OnSettingsWriteCallback BluetoothHandler::getOnSettingsWriteCallback() const {
    return this->_onSettingsWriteCallback;
}

OnConnectCallback BluetoothHandler::getOnConnectCallback() const {
    return this->_onConnectCallback;
}

OnDisconnectCallback BluetoothHandler::getOnDisconnectCallback() const {
    return this->_onDisconnectCallback;
}

bool BluetoothHandler::isConnected() const{
    return this->_isConnected;
}

void BluetoothHandler::setConnectionState(bool connectionState){
    this->_isConnected = connectionState;
}

// This BLE library version (framework-arduinoespressif32 3.3.8, BLE lib 3.3.8) does
// expose the negotiated per-connection MTU: BLEServer::getPeerMTU(conn_id), declared
// in BLEServer.h:163. Use it when a peer is actually connected; fall back to a fixed
// conservative size otherwise (the bluedroid implementation looks the conn_id up in a
// map that is only populated while connected).
size_t BluetoothHandler::usableChunkSize() const {
    BLEServer* pServer = BLEDevice::getServer();
    if (pServer != nullptr && pServer->getConnectedCount() > 0) {
        uint16_t mtu = pServer->getPeerMTU(pServer->getConnId());
        if (mtu > 23) {
            return (size_t)(mtu - 3); // 3 bytes ATT opcode + handle overhead
        }
    }
    return FALLBACK_CHUNK_SIZE;
}

void BluetoothHandler::notifyTelemetry(const uint8_t* data, size_t length) {
    if (_telemetryChar == nullptr || !this->_isConnected) {
        return;
    }

    const size_t chunkSize = usableChunkSize();
    for (size_t offset = 0; offset < length; offset += chunkSize) {
        size_t n = min(chunkSize, length - offset);
        _telemetryChar->setValue((uint8_t*)(data + offset), n);
        _telemetryChar->notify();
        if (offset + n < length) {
            delay(RELAY_CHUNK_PACING_MS);
        }
    }
}

void BluetoothHandler::indicateSettings(const uint8_t* data, size_t length) {
    if (_settingsChar == nullptr || !this->_isConnected) {
        return;
    }

    const size_t chunkSize = usableChunkSize();
    for (size_t offset = 0; offset < length; offset += chunkSize) {
        size_t n = min(chunkSize, length - offset);
        _settingsChar->setValue((uint8_t*)(data + offset), n);
        _settingsChar->indicate();
        if (offset + n < length) {
            delay(RELAY_CHUNK_PACING_MS);
        }
    }
}
