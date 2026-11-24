#include "AxisDriver/AxisDriver.h"

AxisDriver::AxisDriver(Axis& axis, int center, int deadZone, int fullDeflection)
    : axis_(axis),
      center_(center),
      deadZone_((deadZone >= 0) ? deadZone : 0),
      fullDeflection_((fullDeflection > 0) ? fullDeflection : 1) {}

namespace {
// Shape the ramp so the response is gentle near the centre and steepens toward
// full deflection. Input and output are fixed-point 0..256.
//
// This is t^1.8 written as (4*t^2 + t) / 5: a weighted blend of t^2 and t whose
// effective exponent is (2*4 + 1)/5 = 1.8. Doing it this way keeps the curve
// exact enough to be indistinguishable from a real power function while using
// nothing but 16-bit integer multiply on an AVR with no FPU.
uint16_t easeIn(uint16_t t256) {
    const uint32_t t2 = ((uint32_t)t256 * t256) / 256u;   // t^2 in 0..256
    return (uint16_t)((t2 * 4u + t256) / 5u);
}
}  // namespace

uint16_t AxisDriver::speedDelayFor(int stickValue) const {
    // Distance from rest. abs() keeps both directions identical, which is the
    // whole point of a symmetric pan/tilt stick.
    const int strength = abs(stickValue - center_);

    if (strength <= deadZone_) {
        return kDelaySlowest;          // caller treats this as "stopped"
    }

    const int span = fullDeflection_ - deadZone_;
    if (span <= 0) {
        return kDelaySlowest;          // degenerate settings: stay gentle
    }

    const int excess = strength - deadZone_;
    if (excess >= span) {
        return kDelayFastest;          // fully pushed: top speed
    }

    // t in 0..256, then curve it, then interpolate the delay.
    const uint16_t t256    = (uint16_t)(((uint32_t)excess * 256u) / (uint32_t)span);
    const uint16_t curved  = easeIn(t256);
    const int32_t  range   = (int32_t)kDelaySlowest - (int32_t)kDelayFastest;
    const int32_t  drop    = (range * (int32_t)curved) / 256;

    return (uint16_t)((int32_t)kDelaySlowest - drop);
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
    // Returning false means "not time for the next degree yet", which is normal
    // and not a stall.
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
