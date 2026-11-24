#include "AxisDriver/AxisDriver.h"

AxisDriver::AxisDriver(Axis& axis, int center, int deadZone)
    : axis_(axis), center_(center), deadZone_(deadZone) {
    // A dead zone narrower than the tier thresholds keeps every band reachable;
    // a negative one would make the stick step with it barely touched.
    if (deadZone_ < 0) {
        deadZone_ = 0;
    }
}

uint16_t AxisDriver::speedDelayFor(int stickValue) const {
    const int strength = abs(stickValue - center_);   // distance from rest

    if (strength > kTierMed) {
        return kDelayFast;                // pushed hard
    }
    if (strength > kTierSlow) {
        return kDelayMedium;
    }
    return kDelaySlow;                    // just outside the dead zone
}

bool AxisDriver::update(int stickValue) {
    const int delta = stickValue - center_;   // <0 below rest, >0 above

    if (delta < -deadZone_) {
        return axis_.rotateCounterClockwise(speedDelayFor(stickValue));
    }
    if (delta > deadZone_) {
        return axis_.rotateClockwise(speedDelayFor(stickValue));
    }
    return false;                             // centred: hold position
}
