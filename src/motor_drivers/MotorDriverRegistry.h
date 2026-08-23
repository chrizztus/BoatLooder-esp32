#ifndef MOTOR_DRIVER_REGISTRY_H
#define MOTOR_DRIVER_REGISTRY_H

#include "MotorDriverBackend.h"

// Looks up the (single, static) backend instance for a given driver type.
// Adding a new driver: implement MotorDriverBackend, add one case here.
MotorDriverBackend* getMotorDriverBackend(MotorDriverType type);

#endif // MOTOR_DRIVER_REGISTRY_H
