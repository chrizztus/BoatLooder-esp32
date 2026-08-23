#include "Bts7960DriverBackend.h"
#include "Logger.h"

#define MOTOR_PWM1_PIN 21
#define MOTOR_PWM2_PIN 22
#define MOTOR_EN_PIN 19

void Bts7960DriverBackend::begin() {
  LOG_INFO("Initializing pins for BTS7960 Driver");
  pinMode(MOTOR_EN_PIN, OUTPUT);
  pinMode(MOTOR_PWM1_PIN, OUTPUT);
  pinMode(MOTOR_PWM2_PIN, OUTPUT);

  digitalWrite(MOTOR_EN_PIN, HIGH);

  ledcAttachChannel(MOTOR_PWM1_PIN, 20000, 9, 0);
  ledcAttachChannel(MOTOR_PWM2_PIN, 20000, 9, 1);
  ledcWrite(MOTOR_PWM1_PIN, 0);
  ledcWrite(MOTOR_PWM2_PIN, 0);
}

void Bts7960DriverBackend::end() {
  ledcWrite(MOTOR_PWM1_PIN, 0);
  ledcWrite(MOTOR_PWM2_PIN, 0);
  ledcDetach(MOTOR_PWM1_PIN);
  ledcDetach(MOTOR_PWM2_PIN);
  digitalWrite(MOTOR_EN_PIN, LOW);
}

void Bts7960DriverBackend::setSpeed(int motorPulseUs) {
  int speed;

  if (motorPulseUs > (PWM_MID + MOTOR_DEADBAND)) {
    ledcWrite(MOTOR_PWM1_PIN, 0);
    speed = map(motorPulseUs, PWM_MID, PWM_MAX, 0, 512);
    speed = constrain(speed, 0, 512);
    ledcWrite(MOTOR_PWM2_PIN, speed);
  } else if (motorPulseUs < (PWM_MID - MOTOR_DEADBAND)) {
    ledcWrite(MOTOR_PWM2_PIN, 0);
    speed = map(motorPulseUs, PWM_MID, PWM_MIN, 0, 512);
    speed = constrain(speed, 0, 512);
    ledcWrite(MOTOR_PWM1_PIN, speed);
  } else {
    ledcWrite(MOTOR_PWM1_PIN, 0);
    ledcWrite(MOTOR_PWM2_PIN, 0);
  }
}
