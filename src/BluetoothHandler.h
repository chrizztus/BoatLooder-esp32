#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include "Arduino.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define BOATLUDER BoatLuder@
#define BL_NAME chr!zz+us //  ТОППОФ
#define DEVICE_NAME STR(BOATLUDER) STR(BL_NAME)

// Longest frame the settings channel ever carries (PARAM_VALUE serializes to
// 37 bytes); sized with headroom rather than MAVLINK_MAX_PACKET_LEN so the
// outbound queue stays small.
#define SETTINGS_ACK_MAX_LEN 128
#define SETTINGS_ACK_QUEUE_DEPTH 24

typedef std::function<void(const uint8_t* data, size_t length)> OnWriteCallback;
typedef std::function<void(const uint8_t* data, size_t length)> OnSettingsWriteCallback;
typedef std::function<void()> OnConnectCallback;
typedef std::function<void()> OnDisconnectCallback;

class BluetoothHandler {
public:
    BluetoothHandler();
    void init();

    // Setters
    void setOnWriteCallback(OnWriteCallback callback);
    void setOnSettingsWriteCallback(OnSettingsWriteCallback callback);
    void setOnConnectCallback(OnConnectCallback callback);
    void setOnDisconnectCallback(OnDisconnectCallback callback);

    void setConnectionState(bool connectionState);

    // Getters
    OnWriteCallback getOnWriteCallback() const;
    OnSettingsWriteCallback getOnSettingsWriteCallback() const;
    OnConnectCallback getOnConnectCallback() const;
    OnDisconnectCallback getOnDisconnectCallback() const;

    bool isConnected() const;

    // MAVLink relay out to the app: telemetry via notify, param acks via indicate.
    // Both chunk to the negotiated MTU, see the note in the .cpp.
    void notifyTelemetry(const uint8_t* data, size_t length);
    void indicateSettings(const uint8_t* data, size_t length);

private:
    // usable payload per PDU for the current connection (negotiated MTU - 3 ATT bytes)
    size_t usableChunkSize() const;

    bool _isConnected;
    // indicate() blocks until the phone confirms (or times out), so it must
    // never run on the UART parse path: during a PARAM_REQUEST_LIST dump that
    // stalls the reader and overflows the 921600-baud RX buffer. Queue the
    // frames and let one dedicated task send them, one confirmation at a time.
    struct SettingsAck {
        uint16_t length;
        uint8_t data[SETTINGS_ACK_MAX_LEN];
    };
    QueueHandle_t _settingsAckQueue;
    static void settingsAckTask(void* arg);
    void sendSettingsIndication(const uint8_t* data, size_t length);

    BLECharacteristic* _telemetryChar;
    BLECharacteristic* _settingsChar;
    OnWriteCallback _onWriteCallback;
    OnSettingsWriteCallback _onSettingsWriteCallback;
    OnConnectCallback _onConnectCallback;
    OnDisconnectCallback _onDisconnectCallback;
};

#endif // BLUETOOTH_HANDLER_H
