#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define BOATLUDER BoatLuder@
#define BL_NAME chr!zz+us //  ТОППОФ
#define DEVICE_NAME STR(BOATLUDER) STR(BL_NAME)

typedef std::function<void(const uint8_t* data, size_t length)> OnWriteCallback;
typedef std::function<void()> OnConnectCallback;
typedef std::function<void()> OnDisconnectCallback;

class BluetoothHandler {
public:
    BluetoothHandler();
    void init();

    // Setters
    void setOnWriteCallback(OnWriteCallback callback);
    void setOnConnectCallback(OnConnectCallback callback);
    void setOnDisconnectCallback(OnDisconnectCallback callback);

    void setConnectionState(bool connectionState);

    // Getters
    OnWriteCallback getOnWriteCallback() const;
    OnConnectCallback getOnConnectCallback() const;
    OnDisconnectCallback getOnDisconnectCallback() const;

    bool isConnected() const;

private:
    bool _isConnected;
    OnWriteCallback _onWriteCallback;
    OnConnectCallback _onConnectCallback;
    OnDisconnectCallback _onDisconnectCallback;
};

#endif // BLUETOOTH_HANDLER_H
