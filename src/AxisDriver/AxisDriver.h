#pragma once

#include <Arduino.h>
#include "Axis/Axis.h"

// Turns a joystick reading into axis motion.
//
// This is the "policy" half of driving a servo; Axis is the "mechanism" half.
// Keeping them apart means the same Axis can be driven by a stick, a scripted
// move or a saved pose without any of those knowing about each other.
//
// The stick rests near `center` (512 on a 10-bit ADC). The further it is pushed
// away from the centre, the faster the axis is asked to move:
//
//   |delta| <= deadZone        -> hold still
//   |delta| <= tier2           -> slow   (kDelaySlow   ms per degree)
//   |delta| <= tier3           -> medium (kDelayMedium ms per degree)
//   |delta|  > tier3           -> fast   (kDelayFast   ms per degree)
//
// update() advances the axis by at most ONE degree per call, so loop() stays
// responsive and the I2C receive path keeps running while the servo moves.
class AxisDriver {
public:
    static constexpr int kDefaultCenter   = 512;   // ADC value at rest
    static constexpr int kDefaultDeadZone = 30;

    AxisDriver(Axis& axis,
               int center  = kDefaultCenter,
               int deadZone = kDefaultDeadZone);

    // Feed the latest joystick reading. Returns true when the axis moved.
    bool update(int stickValue);

    // Delay chosen for a reading (exposed for tuning and debugging).
    uint16_t speedDelayFor(int stickValue) const;

    void setCenter(int center) { center_ = center; }
    void setDeadZone(int deadZone) { deadZone_ = deadZone; }
    int  getCenter() const { return center_; }

private:
    Axis& axis_;
    int center_;
    int deadZone_;

    static constexpr int kTierSlow   = 200;   // 200..599  -> medium
    static constexpr int kTierMedium = 600;   // 600..849  -> fast
    static constexpr int kTierFast   = 850;   // 850+      -> fastest

    static constexpr uint16_t kDelaySlow   = 8;
    static constexpr uint16_t kDelayMedium = 4;
    static constexpr uint16_t kDelayFast   = 1;
};
