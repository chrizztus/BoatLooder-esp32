#include "MotorDriverRegistry.h"
#include "PololuDriverBackend.h"
#include "Bts7960DriverBackend.h"

static PololuDriverBackend pololuBackend;
static Bts7960DriverBackend bts7960Backend;

MotorDriverBackend* getMotorDriverBackend(MotorDriverType type) {
  switch (type) {
    case MotorDriverType::Pololu18v17:
      return &pololuBackend;
    case MotorDriverType::Bts7960:
    default:
      return &bts7960Backend;
  }
}
