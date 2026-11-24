#include "AxisDriver/AxisDriver.h"

AxisDriver::AxisDriver(Axis& axis, int center, int deadZone)
    : axis_(axis), center_(center), deadZone_(deadZone) {}

uint16_t AxisDriver::speedDelayFor(int stickValue) const {
    const int strength = abs(stickValue - center_);   // distance from rest

    if (strength > kTierFast) {
        return kDelayFast;
    }
    if (strength > kTierMedium) {
        return kDelayMedium;
    }
    return kDelaySlow;
}

bool AxisDriver::update(int stickValue) {
    const int delta = stickValue - center_;   // <0 below rest, >0 above

    if (delta < -deadZone_) {
        axis_.rotateCounterClockwise(speedDelayFor(stickValue));
        return true;
    }
    if (delta > deadZone_) {
        axis_.rotateClockwise(speedDelayFor(stickValue));
        return true;
    }
    return false;                             // centred: hold position
}

