#include "Mavlink.h"

#define HZ_TO_US(hz) (1000000 / (hz))


void Mavlink::init(){
  _mavSerial.begin(MAVLINK_UART_BAUDRATE, SERIAL_8N1, MAVLINK_UART_RX, MAVLINK_UART_TX);

  // Attach interrupt to the UART RX pin
  _servoOutSteering = 1500;
  _servoOutThrottle = 1500;

  _rcChannelPulses = new uint16_t[_rcChannels];

  for(uint8_t i; i < _rcChannels; ++i){
    _rcChannelPulses[i] = 800;
  }

  // // set streaming rates
  requestMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, HZ_TO_US(MAVLINK_SERVO_OUTPUT_RAW_INTERVAL_HZ));
  requestMessageInterval(MAVLINK_MSG_ID_HEARTBEAT, HZ_TO_US(MAVLINK_HEARTBEAT_INTERVAL_HZ));
  requestMessageInterval(65, -1);
  Serial.println("Mavlink initilized.");
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
  return (now - _lastHeartbeat) < MAVLINK_HEARTBEAT_TIMEOUT_MS;
}

void Mavlink::processReceivedPacket() {
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
        Serial.printf("Received Message ID: %d\n", _msg.msgid);
        if (_msg.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) {
            mavlink_servo_output_raw_t servo_output;
            mavlink_msg_servo_output_raw_decode(&_msg, &servo_output);

            _servoOutThrottle = servo_output.servo3_raw;
            _servoOutSteering = servo_output.servo1_raw;
        }

        if (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            _lastHeartbeat = millis();
        }
    }
}

