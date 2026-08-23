#ifndef MOTOR_DRIVER_BACKEND_H
#define MOTOR_DRIVER_BACKEND_H

#include "Arduino.h"

// Shared PWM range every backend's setSpeed() interprets the same way --
// moved here from main.cpp so both main.cpp and every backend .cpp can see
// them without main.cpp exposing its own header.
#define PWM_MID 1500
#define PWM_MIN 1100
#define PWM_MAX 1900
#define MOTOR_DEADBAND 20

// Wire-visible id (VesselConfig's SET_DRIVER frame, vessel_info's
// "driver=" field) -- keep in sync with boatlooder-app's MotorDriver enum
// by hand, same caveat OtaUpdate.h's OtaError has.
enum class MotorDriverType : uint8_t {
  Pololu18v17 = 0,
  Bts7960 = 1,
};

// One motor driver board's pin setup and PWM control, behind a common
// interface so main.cpp's thrustControlTask can hold whichever one is
// currently selected and switch at runtime -- see MotorDriverRegistry.
// Adding a new driver board later means writing one new class here and
// registering it, nothing else in this file or main.cpp needs to change.
class MotorDriverBackend {
public:
  virtual ~MotorDriverBackend() {}

  virtual MotorDriverType type() const = 0;

  // Display label -- single source of truth, reused by VesselConfig when
  // composing vessel_info's "driver=" field so the label only lives here.
  virtual const char* name() const = 0;

  // pinMode/ledcAttachChannel etc. Must leave the motor at rest.
  virtual void begin() = 0;

  // Quiesces output and releases whatever begin() attached (ledcDetach on
  // every pin it touched). Called before switching to a different backend
  // -- the outgoing backend must leave the shared pins in a state the
  // incoming backend's begin() can safely reconfigure from scratch.
  virtual void end() = 0;

  // Same pulse contract the old setMotorSpeed() had: a value centered on
  // PWM_MID, +-MOTOR_DEADBAND treated as neutral.
  virtual void setSpeed(int motorPulseUs) = 0;
};

#endif // MOTOR_DRIVER_BACKEND_H
