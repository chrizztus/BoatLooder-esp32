#include "Mavlink.h"
#include "Logger.h"

#define HZ_TO_US(hz) (1000000 / (hz))


void Mavlink::init(){
  _mavSerial.begin(MAVLINK_UART_BAUDRATE, SERIAL_8N1, MAVLINK_UART_RX, MAVLINK_UART_TX);

  _servoOutSteering = 1500;
  _servoOutThrottle = 1500;

  _rcChannelPulses = new uint16_t[_rcChannels];

  for(uint8_t i = 0; i < _rcChannels; ++i){
    _rcChannelPulses[i] = 800;
  }

  LOG_INFO("Mavlink initilized.");
}

// Every message we ask the flight controller to stream. SERVO_OUTPUT_RAW drives
// the local control loop (getThrottlePulseUs / getSteeringPulseUs); the rest are
// relayed to the app over the BLE telemetry characteristic. PARAM_VALUE is
// deliberately absent: it is response-driven, not a periodic stream.
static const struct {
  uint32_t msgId;
  uint32_t intervalUs;
} STREAMED_MESSAGES[] = {
  { MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,    HZ_TO_US(MAVLINK_SERVO_OUTPUT_RAW_INTERVAL_HZ) },
  { MAVLINK_MSG_ID_HEARTBEAT,           HZ_TO_US(MAVLINK_HEARTBEAT_INTERVAL_HZ) },
  { MAVLINK_MSG_ID_VFR_HUD,             HZ_TO_US(MAVLINK_VFR_HUD_INTERVAL_HZ) },
  { MAVLINK_MSG_ID_GPS_RAW_INT,         HZ_TO_US(MAVLINK_GPS_RAW_INT_INTERVAL_HZ) },
  { MAVLINK_MSG_ID_SYS_STATUS,          HZ_TO_US(MAVLINK_SYS_STATUS_INTERVAL_HZ) },
  { MAVLINK_MSG_ID_EKF_STATUS_REPORT,   HZ_TO_US(MAVLINK_EKF_STATUS_REPORT_INTERVAL_HZ) },
  { MAVLINK_MSG_ID_GLOBAL_POSITION_INT, HZ_TO_US(MAVLINK_GLOBAL_POSITION_INT_INTERVAL_HZ) },
  { MAVLINK_MSG_ID_VIBRATION,           MAVLINK_VIBRATION_INTERVAL_US },
  { MAVLINK_MSG_ID_HOME_POSITION,       MAVLINK_HOME_POSITION_INTERVAL_US }
};

static const uint8_t STREAMED_MESSAGE_COUNT =
    sizeof(STREAMED_MESSAGES) / sizeof(STREAMED_MESSAGES[0]);

void Mavlink::setupStreamingRates(){
  _streamSeenMask = 0;
  _streamRetryRounds = 0;
  _streamGapReported = false;
  _lastStreamCheck = millis();

  for (uint8_t i = 0; i < STREAMED_MESSAGE_COUNT; ++i) {
    requestMessageInterval(STREAMED_MESSAGES[i].msgId, STREAMED_MESSAGES[i].intervalUs);
    delay(100);
  }
}

void Mavlink::markStreamSeen(uint32_t msgid) {
  for (uint8_t i = 0; i < STREAMED_MESSAGE_COUNT; ++i) {
    if (STREAMED_MESSAGES[i].msgId == msgid) {
      _streamSeenMask |= (1UL << i);
      return;
    }
  }
}

// Observed on hardware: of nine requests sent back to back at startup only six
// were ever acknowledged, and exactly those six streamed -- EKF_STATUS_REPORT,
// GPS_RAW_INT and HOME_POSITION never arrived, leaving the app's EKF tile
// permanently "NO DATA". A single fire-and-forget round is not enough, so keep
// asking for whatever has not shown up.
void Mavlink::ensureStreamsFlowing() {
  if (!haveHeartbeat()) {
    return;
  }

  const uint32_t allSeen = (STREAMED_MESSAGE_COUNT >= 32)
      ? 0xFFFFFFFFUL
      : ((1UL << STREAMED_MESSAGE_COUNT) - 1);
  if (_streamSeenMask == allSeen) {
    return;
  }

  unsigned long now = millis();
  if (now - _lastStreamCheck < MAVLINK_STREAM_RECHECK_MS) {
    return;
  }
  _lastStreamCheck = now;

  if (_streamRetryRounds >= MAVLINK_STREAM_MAX_RETRY_ROUNDS) {
    // Give up quietly rather than talking to the flight controller forever.
    // Some messages legitimately never arrive: HOME_POSITION is not sent until
    // a home position exists, which needs a GPS fix.
    if (!_streamGapReported) {
      _streamGapReported = true;
      for (uint8_t i = 0; i < STREAMED_MESSAGE_COUNT; ++i) {
        if (!(_streamSeenMask & (1UL << i))) {
          LOG_WARNF("Stream msgid %u never arrived after %u retries\n",
                    (unsigned)STREAMED_MESSAGES[i].msgId,
                    (unsigned)MAVLINK_STREAM_MAX_RETRY_ROUNDS);
        }
      }
    }
    return;
  }

  _streamRetryRounds++;
  for (uint8_t i = 0; i < STREAMED_MESSAGE_COUNT; ++i) {
    if (!(_streamSeenMask & (1UL << i))) {
      LOG_INFOF("Re-requesting stream msgid %u (round %u)\n",
                (unsigned)STREAMED_MESSAGES[i].msgId, (unsigned)_streamRetryRounds);
      requestMessageInterval(STREAMED_MESSAGES[i].msgId, STREAMED_MESSAGES[i].intervalUs);
      delay(20);
    }
  }
}

void Mavlink::sendRcOverrides(const uint16_t *pulses){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the RC_CHANNELS_OVERRIDE message
    mavlink_msg_rc_channels_override_pack(MAVLINK_LOCAL_SYSTEM_ID,
                                          MAVLINK_LOCAL_COMPONENT_ID,
                                          &msg,
                                          MAVLINK_TARGET_SYSTEM_ID,
                                          MAVLINK_TARGET_COMPONENT_ID,
                                          pulses[0],
                                          pulses[1],
                                          pulses[2],
                                          pulses[3],
                                          pulses[4],
                                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the command
    _mavSerial.write(buf, len);
}

uint16_t Mavlink::getThrottlePulseUs(void){
  return _servoOutThrottle;
}

uint16_t Mavlink::getSteeringPulseUs(void){
  return _servoOutSteering;
}

bool Mavlink::haveHeartbeat(void){
  unsigned long now = millis(); 
  return (_lastHeartbeat != 0) && (now - _lastHeartbeat) < MAVLINK_HEARTBEAT_TIMEOUT_MS;
}

void Mavlink::setOnTelemetryRelayCallback(OnRelayCallback callback) {
    this->_onTelemetryRelayCallback = callback;
}

void Mavlink::setOnSettingsAckRelayCallback(OnRelayCallback callback) {
    this->_onSettingsAckRelayCallback = callback;
}

void Mavlink::processReceivedPackets() {
    while (_mavSerial.available()) {
         handleReceivedByte(_mavSerial.read());
    }
}

// private
void Mavlink::requestMessageInterval(uint16_t message_id, uint32_t interval_us) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the command to request message interval
  mavlink_msg_command_long_pack(MAVLINK_LOCAL_SYSTEM_ID,
                                MAVLINK_LOCAL_COMPONENT_ID, 
                                &msg,
                                MAVLINK_TARGET_SYSTEM_ID, 
                                MAVLINK_TARGET_COMPONENT_ID, 
                                MAV_CMD_SET_MESSAGE_INTERVAL, 0, 
                                message_id,
                                interval_us,
                                0, 0, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the command
  _mavSerial.write(buf, len);
}

void Mavlink::handleReceivedByte(uint8_t byte) {
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &_msg, &_status)) {
        LOG_DEBUGF("Received Message ID: %d\n", _msg.msgid);
        markStreamSeen(_msg.msgid);
        if (_msg.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) {
            mavlink_servo_output_raw_t servo_output;
            mavlink_msg_servo_output_raw_decode(&_msg, &servo_output);

            _servoOutThrottle = servo_output.servo3_raw;
            _servoOutSteering = servo_output.servo1_raw;
        }

        if (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            _lastHeartbeat = millis();
        }

        // Relay is purely msgid-driven: the already-validated frame is re-serialized
        // and forwarded verbatim, no per-message decode needed on this side.
        RelayChannel channel = classifyRelay(_msg.msgid);
        if (channel != RelayChannel::NONE) {
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buf, &_msg);
            if (channel == RelayChannel::TELEMETRY && _onTelemetryRelayCallback) {
                _onTelemetryRelayCallback(buf, len);
            } else if (channel == RelayChannel::SETTINGS_ACK && _onSettingsAckRelayCallback) {
                _onSettingsAckRelayCallback(buf, len);
            }
        }
    }
}

// BLE settings writes come in on their own parser channel (MAVLINK_COMM_1) -- the
// mavlink C library keys parser state by channel, so reusing MAVLINK_COMM_0 here
// would corrupt both streams.
void Mavlink::handleBleSettingsByte(uint8_t byte) {
    if (mavlink_parse_char(MAVLINK_COMM_1, byte, &_bleMsg, &_bleStatus)) {
        if (!isAllowedFromApp(_bleMsg.msgid)) {
            LOG_WARNF("Dropped disallowed msgid %d from BLE settings channel\n", _bleMsg.msgid);
            return;
        }
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &_bleMsg);
        _mavSerial.write(buf, len);
    }
}

RelayChannel Mavlink::classifyRelay(uint16_t msgid) {
    switch (msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
        case MAVLINK_MSG_ID_VFR_HUD:
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        case MAVLINK_MSG_ID_SYS_STATUS:
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
        case MAVLINK_MSG_ID_VIBRATION:
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        case MAVLINK_MSG_ID_HOME_POSITION:
            return RelayChannel::TELEMETRY;
        case MAVLINK_MSG_ID_PARAM_VALUE:
            return RelayChannel::SETTINGS_ACK;
        default:
            return RelayChannel::NONE;
    }
}

// Security boundary, not a convenience filter: this path forwards whatever the app
// sends straight to the flight controller, so it is a tight allowlist. Anything that
// could arm motors or change mode must go through the RC-override control path.
bool Mavlink::isAllowedFromApp(uint16_t msgid) {
    switch (msgid) {
        case MAVLINK_MSG_ID_PARAM_SET:
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            return true;
        default:
            return false;
    }
}

