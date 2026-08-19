#pragma once

#include "Arduino.h"
#include "mavlink/v2.0/common/mavlink.h"

#define MAVLINK_HEARTBEAT_INTERVAL_HZ 1
#define MAVLINK_SERVO_OUTPUT_RAW_INTERVAL_HZ 10

#define MAVLINK_HEARTBEAT_TIMEOUT_MS 5000

#define MAVLINK_TARGET_SYSTEM_ID 1
#define MAVLINK_TARGET_COMPONENT_ID 0
#define MAVLINK_LOCAL_SYSTEM_ID 255
#define MAVLINK_LOCAL_COMPONENT_ID 0

#define MAVLINK_UART_BAUDRATE 921600
#define MAVLINK_UART_RX 13
#define MAVLINK_UART_TX 14

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

    // functions
    void requestMessageInterval(uint16_t, uint32_t);
    static void onUartRx(void* arg);
    void handleReceivedByte(uint8_t byte);

public:
    Mavlink(uint8_t numChannels, uint8_t mavUart) : _rcChannels(numChannels), _mavSerial(mavUart) {}
    void init();
    void setupStreamingRates();
    void sendRcOverrides(const uint16_t* pulses);
    uint16_t getThrottlePulseUs(void);
    uint16_t getSteeringPulseUs(void);
    bool haveHeartbeat(void);
    void processReceivedPackets();
};

