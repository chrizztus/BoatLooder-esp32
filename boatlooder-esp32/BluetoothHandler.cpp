#include "BluetoothHandler.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class BoatLuderCallbacks: public BLECharacteristicCallbacks {
    BluetoothHandler* handler;
public:
    BoatLuderCallbacks(BluetoothHandler* handler) : handler(handler) {}

    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue().c_str();
      if (!value.empty() && handler->getOnWriteCallback()) {
          handler->getOnWriteCallback()((const uint8_t*)value.data(), value.length());
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
            handler->setConnectionState(true);
            handler->getOnDisconnectCallback()();
        }
        pServer->getAdvertising()->start();  // Restart advertising
    }
};

BluetoothHandler::BluetoothHandler() {}

void BluetoothHandler::init() {
    Serial.println("BT HANLDER INIT :: START");
    BLEDevice::init(DEVICE_NAME);
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
    pService->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    Serial.println("BT HANDLER INIT :: DONE");

    this->_isConnected = false;
  }

// Setter implementations
void BluetoothHandler::setOnWriteCallback(OnWriteCallback callback) {
    Serial.println("BT HANLDER WRITE CB");
    this->_onWriteCallback = callback;
}

void BluetoothHandler::setOnConnectCallback(OnConnectCallback callback) {
    Serial.println("BT HANLDER CONNECT CB");
    this->_onConnectCallback = callback;
}

void BluetoothHandler::setOnDisconnectCallback(OnDisconnectCallback callback) {
    Serial.println("BT HANLDER DISCONNECT CB");
    this->_onDisconnectCallback = callback;
}

// Getter implementations
OnWriteCallback BluetoothHandler::getOnWriteCallback() const {
    return this->_onWriteCallback;
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
