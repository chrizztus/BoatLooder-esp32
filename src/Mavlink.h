#pragma once

#include "Arduino.h"
#include "mavlink/v2.0/common/mavlink.h"

#define MAVLINK_HEARTBEAT_INTERVAL_HZ 1
#define MAVLINK_SERVO_OUTPUT_RAW_INTERVAL_HZ 10

// streaming rates for the messages relayed to the app over BLE
#define MAVLINK_VFR_HUD_INTERVAL_HZ 2
#define MAVLINK_GPS_RAW_INT_INTERVAL_HZ 1
#define MAVLINK_SYS_STATUS_INTERVAL_HZ 1
#define MAVLINK_EKF_STATUS_REPORT_INTERVAL_HZ 1
#define MAVLINK_GLOBAL_POSITION_INT_INTERVAL_HZ 1
// sub-Hz rates are expressed directly in microseconds -- HZ_TO_US() is integer math
#define MAVLINK_VIBRATION_INTERVAL_US 2000000      // 0.5 Hz
#define MAVLINK_HOME_POSITION_INTERVAL_US 5000000  // 0.2 Hz

#define MAVLINK_HEARTBEAT_TIMEOUT_MS 5000

#define MAVLINK_TARGET_SYSTEM_ID 1
#define MAVLINK_TARGET_COMPONENT_ID 0
#define MAVLINK_LOCAL_SYSTEM_ID 255
#define MAVLINK_LOCAL_COMPONENT_ID 0

#define MAVLINK_UART_BAUDRATE 921600
#define MAVLINK_UART_RX 13
#define MAVLINK_UART_TX 14

// EKF_STATUS_REPORT lives in the ardupilotmega dialect, which is not vendored here
// (include/mavlink/v2.0 only carries common/minimal/standard). ArduPilot emits it and
// both sides of the BLE contract expect msgid 193, so define just the id -- the relay
// never decodes the payload, it only needs to recognise the number.
#ifndef MAVLINK_MSG_ID_EKF_STATUS_REPORT
#define MAVLINK_MSG_ID_EKF_STATUS_REPORT 193
#endif

// where a received message gets forwarded, if anywhere
enum class RelayChannel { NONE, TELEMETRY, SETTINGS_ACK };

typedef std::function<void(const uint8_t* data, size_t length)> OnRelayCallback;

class Mavlink {
private:
    // variables
    volatile uint16_t _servoOutThrottle, _servoOutSteering;
    unsigned long _lastHeartbeat;
    // TODO: dead — allocated and filled in init(), never read. sendRcOverrides()
    // packs the caller's array instead. Either drop it, or move ownership of the
    // failsafe defaults here from main.cpp's initRcChannels(). See TODO.md.
    uint16_t *_rcChannelPulses;
    uint8_t _rcChannels;
    HardwareSerial _mavSerial;

    // for mavlink message parser
    mavlink_message_t _msg;
    mavlink_status_t _status;

    // second, independent parser state -- the BLE settings stream and the UART
    // stream are unrelated byte streams and must not share _msg/_status
    mavlink_message_t _bleMsg;
    mavlink_status_t _bleStatus;

    OnRelayCallback _onTelemetryRelayCallback;
    OnRelayCallback _onSettingsAckRelayCallback;

    // functions
    void requestMessageInterval(uint16_t, uint32_t);
    static void onUartRx(void* arg);
    void handleReceivedByte(uint8_t byte);
    RelayChannel classifyRelay(uint16_t msgid);
    bool isAllowedFromApp(uint16_t msgid);

public:
    Mavlink(uint8_t numChannels, uint8_t mavUart) : _rcChannels(numChannels), _mavSerial(mavUart) {}
    void init();
    void setupStreamingRates();
    void sendRcOverrides(const uint16_t* pulses);
    uint16_t getThrottlePulseUs(void);
    uint16_t getSteeringPulseUs(void);
    bool haveHeartbeat(void);
    void processReceivedPackets();

    // Setters
    void setOnTelemetryRelayCallback(OnRelayCallback callback);
    void setOnSettingsAckRelayCallback(OnRelayCallback callback);

    // BLE -> UART direction (param requests written by the app)
    void handleBleSettingsByte(uint8_t byte);
};

