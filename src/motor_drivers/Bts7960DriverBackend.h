#ifndef BTS7960_DRIVER_BACKEND_H
#define BTS7960_DRIVER_BACKEND_H

#include "MotorDriverBackend.h"

// Dual-PWM H-bridge (BTS7960) -- one channel per direction, plus an
// enable pin held high whenever this backend is active.
class Bts7960DriverBackend : public MotorDriverBackend {
public:
  MotorDriverType type() const override { return MotorDriverType::Bts7960; }
  const char* name() const override { return "BTS7960"; }

  void begin() override;
  void end() override;
  void setSpeed(int motorPulseUs) override;
};

#endif // BTS7960_DRIVER_BACKEND_H
