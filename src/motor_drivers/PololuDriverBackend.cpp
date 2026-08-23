#include "PololuDriverBackend.h"
#include "Logger.h"

#define MOTOR_DIR_PIN 21
#define MOTOR_PWM_PIN 19

void PololuDriverBackend::begin() {
  LOG_INFO("Initializing pins for POLULU 18v17 Driver");
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  ledcAttachChannel(MOTOR_PWM_PIN, 20000, 9, 0);
  ledcWrite(MOTOR_PWM_PIN, 0);
}

void PololuDriverBackend::end() {
  ledcWrite(MOTOR_PWM_PIN, 0);
  ledcDetach(MOTOR_PWM_PIN);
}

void PololuDriverBackend::setSpeed(int motorPulseUs) {
  bool direction = motorPulseUs > PWM_MID;
  int speed;

  digitalWrite(MOTOR_DIR_PIN, direction ? HIGH : LOW);
  if (direction) {
    speed = map(motorPulseUs, PWM_MID, PWM_MAX, 0, 512);
  } else {
    speed = map(motorPulseUs, PWM_MID, PWM_MIN, 0, 512);
  }
  speed = constrain(speed, 0, 512);
  if (motorPulseUs > (PWM_MID - MOTOR_DEADBAND) && motorPulseUs < (PWM_MID + MOTOR_DEADBAND)) {
    speed = 0;
  }
  ledcWrite(MOTOR_PWM_PIN, speed);
}
