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
#define SETTINGS_ACK_MAX_LEN 64
// Deep on purpose. ArduRover answers PARAM_REQUEST_LIST with ~1000 frames
// back to back over a 921600-baud UART, while BLE can only clear roughly one
// indication per connection interval. A shallow queue simply discards that
// burst, and PARAM_REQUEST_LIST never retransmits, so every dropped frame had
// to be chased individually afterwards. 384 slots x 66 bytes is ~25 KB of the
// ESP32's 320 KB and absorbs most of the burst instead.
#define SETTINGS_ACK_QUEUE_DEPTH 384

typedef std::function<void(const uint8_t* data, size_t length)> OnWriteCallback;
typedef std::function<void(const uint8_t* data, size_t length)> OnSettingsWriteCallback;
typedef std::function<void(const uint8_t* data, size_t length)> OnOtaControlWriteCallback;
typedef std::function<void(const uint8_t* data, size_t length)> OnOtaDataWriteCallback;
typedef std::function<void()> OnConnectCallback;
typedef std::function<void()> OnDisconnectCallback;

class BluetoothHandler {
public:
    BluetoothHandler();
    // [firmwareVersion] becomes ota_version's fixed value -- set once here,
    // never rewritten, since it's only ever the running firmware's own
    // compile-time version (see main.cpp's FIRMWARE_VERSION).
    void init(const char* firmwareVersion);

    // Setters
    void setOnWriteCallback(OnWriteCallback callback);
    void setOnSettingsWriteCallback(OnSettingsWriteCallback callback);
    void setOnOtaControlWriteCallback(OnOtaControlWriteCallback callback);
    void setOnOtaDataWriteCallback(OnOtaDataWriteCallback callback);
    void setOnConnectCallback(OnConnectCallback callback);
    void setOnDisconnectCallback(OnDisconnectCallback callback);

    void setConnectionState(bool connectionState);

    // Getters
    OnWriteCallback getOnWriteCallback() const;
    OnSettingsWriteCallback getOnSettingsWriteCallback() const;
    OnOtaControlWriteCallback getOnOtaControlWriteCallback() const;
    OnOtaDataWriteCallback getOnOtaDataWriteCallback() const;
    OnConnectCallback getOnConnectCallback() const;
    OnDisconnectCallback getOnDisconnectCallback() const;

    bool isConnected() const;

    // MAVLink relay out to the app: telemetry via notify, param acks via indicate.
    // Both chunk to the negotiated MTU, see the note in the .cpp.
    void notifyTelemetry(const uint8_t* data, size_t length);
    void notifySettings(const uint8_t* data, size_t length);

    // ACK/OK/ERROR frames on ota_control -- see OtaUpdate's doc comment for
    // the frame layout. Always tiny (<=2 bytes), never needs the chunking
    // notifyTelemetry/notifySettings do.
    void notifyOtaControl(const uint8_t* data, size_t length);

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
    void sendSettingsNotification(const uint8_t* data, size_t length);

    BLECharacteristic* _telemetryChar;
    BLECharacteristic* _settingsChar;
    // ota_data has no stored pointer -- nothing ever notifies on it, only
    // ota_control does (ACK/OK/ERROR), so only that one needs to survive
    // past init().
    BLECharacteristic* _otaControlChar;
    OnWriteCallback _onWriteCallback;
    OnSettingsWriteCallback _onSettingsWriteCallback;
    OnOtaControlWriteCallback _onOtaControlWriteCallback;
    OnOtaDataWriteCallback _onOtaDataWriteCallback;
    OnConnectCallback _onConnectCallback;
    OnDisconnectCallback _onDisconnectCallback;
};

#endif // BLUETOOTH_HANDLER_H
