#pragma once

#include <Arduino.h>
#include "Axis/Axis.h"

// Turns a joystick reading into axis motion.
//
// This is the "policy" half of driving a servo; Axis is the "mechanism" half.
// Keeping them apart means the same Axis can be driven by a stick, a scripted
// move or a saved pose without any of those knowing about each other.
//
// SPEED MODEL
// -----------
// The stick deflection sets the SPEED of the axis, not its absolute angle.
// The further the stick is pushed from rest, the faster the axis moves, and it
// works identically in both directions:
//
//   |delta| <= deadZone          -> stopped
//   deadZone < |delta| < kFull   -> speed ramps up linearly with deflection
//   |delta| >= kFull             -> maximum speed
//
// where delta = reading - center.
//
// Concretely, with a centre of 512 and kDefaultFullDeflection = 400:
//
//   push to 212  (delta -300) -> ~12 deg/s   (barely creeping)
//   push to 112  (delta -400) -> ~45 deg/s   (noticeably faster)
//   push to   0  (delta -512) -> ~45 deg/s   (capped at maximum)
//
// ...and symmetrically for +300 / +400 / +1023 on the other side.
//
// The maximum speed is deliberately modest. This drives an SG90, a small
// positional servo: it manages roughly 300-600 deg/s at full tilt, but running
// it flat out constantly is what wears it out. ~45 deg/s means a full 160 degree
// sweep takes about 3.5 s, a comfortable camera-panning rate.
//
// update() advances the axis by at most ONE degree per call and paces itself
// with millis(), so loop() never blocks and the I2C receive path keeps running
// while the axis moves.
class AxisDriver {
public:
    static constexpr int kDefaultCenter   = 512;   // nominal ADC midpoint
    static constexpr int kDefaultDeadZone = 25;    // absorbs resting jitter

    // Deflection (ADC counts from centre) at which the axis reaches maximum
    // speed. Anything beyond this stays at the maximum.
    static constexpr int kDefaultFullDeflection = 400;

    AxisDriver(Axis& axis,
               int center  = kDefaultCenter,
               int deadZone = kDefaultDeadZone,
               int fullDeflection = kDefaultFullDeflection);

    // Feed the latest joystick reading. Returns true when the axis moved.
    bool update(int stickValue);

    // Milliseconds to wait before the next 1-degree step for a given reading.
    // Large = slow. Returns the slowest period when inside the dead zone.
    // Exposed for tuning and debugging.
    uint16_t speedDelayFor(int stickValue) const;

    // Current speed in degrees per second for a given reading (0 when stopped).
    uint16_t speedFor(int stickValue) const;

    void setCenter(int center) { center_ = center; }
    void setDeadZone(int deadZone) { deadZone_ = deadZone; }
    void setFullDeflection(int d) { fullDeflection_ = (d > 0) ? d : 1; }
    int  getCenter() const { return center_; }

    // Average `samples` readings of a reader and adopt it as the resting point.
    // Call once at boot with the stick left alone. `read` is any callable
    // returning an int, which keeps this class free of any knowledge about
    // where the value comes from.
    template <typename ReadFn>
    int calibrateFrom(ReadFn read, uint8_t samples = 16) {
        if (samples == 0) {
            return center_;
        }
        int32_t sum = 0;
        for (uint8_t i = 0; i < samples; ++i) {
            sum += read();
            delay(2);                 // let the ADC settle between samples
        }
        const int average = (int)(sum / (int32_t)samples);

        // Only accept a plausible centre. A stick held during boot would
        // otherwise become "the middle" and invert the whole response.
        if (average > kPlausibleMin && average < kPlausibleMax) {
            center_ = average;
        }
        return center_;
    }

    // Re-learn the resting value: call with the stick untouched at boot.
    void calibrate(int stickValue) { center_ = stickValue; }

private:
    Axis& axis_;
    int center_;
    int deadZone_;
    int fullDeflection_;

    // millis() of the last degree actually commanded. Pacing lives here so
    // Axis stays a pure mechanism and loop() never blocks.
    uint32_t lastStepMs_ = 0;

    // A calibrated centre outside this window means the stick was not at rest
    // during calibration; keep the previous value instead.
    static constexpr int kPlausibleMin = 200;
    static constexpr int kPlausibleMax = 824;

    // Step period at the two ends of the speed range, in milliseconds per
    // degree. Slowest = the gentlest nudge just outside the dead zone;
    // fastest = a fully deflected stick.
    //
    //   kDelaySlowest = 80 ms/deg -> ~12.5 deg/s  (delicate aiming)
    //   kDelayFastest = 22 ms/deg -> ~45   deg/s  (full sweep in ~3.5 s)
    static constexpr uint16_t kDelaySlowest = 80;
    static constexpr uint16_t kDelayFastest = 22;
};
