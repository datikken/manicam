#include "AxisDriver/AxisDriver.h"

AxisDriver::AxisDriver(Axis& axis, int center, int deadZone, int fullDeflection)
    : axis_(axis),
      center_(center),
      deadZone_((deadZone >= 0) ? deadZone : 0),
      fullDeflection_((fullDeflection > 0) ? fullDeflection : 1) {}

uint16_t AxisDriver::speedDelayFor(int stickValue) const {
    // Signed distance from rest; abs() keeps the response identical in both
    // directions, which is exactly the requirement.
    const int strength = abs(stickValue - center_);

    if (strength <= deadZone_) {
        return kDelaySlowest;          // stopped: caller checks this separately
    }

    // Map [deadZone_ .. fullDeflection_] onto [kDelaySlowest .. kDelayFastest]
    // as a straight line, so a small nudge creeps and a hard push is fast.
    const int span   = (fullDeflection_ - deadZone_);
    const int excess = strength - deadZone_;

    if (span <= 0 || excess >= span) {
        return kDelayFastest;          // fully pushed (or degenerate settings)
    }

    // Integer interpolation; both endpoints are constants so this is cheap.
    const int32_t drop = (int32_t)(kDelaySlowest - kDelayFastest) * excess;
    const uint16_t delay = (uint16_t)(kDelaySlowest - (drop / span));
    return delay;
}

uint16_t AxisDriver::speedFor(int stickValue) const {
    const int strength = abs(stickValue - center_);
    if (strength <= deadZone_) {
        return 0;                      // stationary
    }
    const uint16_t ms = speedDelayFor(stickValue);
    return (ms > 0) ? (uint16_t)(1000u / ms) : 0;
}

bool AxisDriver::update(int stickValue) {
    const int delta = stickValue - center_;   // <0 below rest, >0 above

    if (delta <= deadZone_ && delta >= -deadZone_) {
        return false;                         // centred: hold position
    }

    // Pace the axis here, not inside Axis: the servo must be commanded at a
    // rate it can physically follow, and loop() must stay free for I2C.
    // Returning false simply means "not time for the next degree yet".
    const uint32_t now = millis();
    if ((uint32_t)(now - lastStepMs_) < (uint32_t)speedDelayFor(stickValue)) {
        return false;
    }
    lastStepMs_ = now;

    // The sign of the deflection picks the direction; the magnitude already
    // picked the speed above. Both directions are symmetric by construction.
    return (delta > 0) ? axis_.rotateClockwise()
                       : axis_.rotateCounterClockwise();
}
