#ifndef POLOLU_DRIVER_BACKEND_H
#define POLOLU_DRIVER_BACKEND_H

#include "MotorDriverBackend.h"

// Single PWM + DIR pin H-bridge (e.g. Pololu 18v17).
class PololuDriverBackend : public MotorDriverBackend {
public:
  MotorDriverType type() const override { return MotorDriverType::Pololu18v17; }
  const char* name() const override { return "Pololu 18V17"; }

  void begin() override;
  void end() override;
  void setSpeed(int motorPulseUs) override;
};

#endif // POLOLU_DRIVER_BACKEND_H
