#pragma once

#include <Arduino.h>
#include <Servo.h>

// One servo-driven axis of the camera rig.
class Axis {
public:
    static constexpr int kMinAngleValue = 0;      // full-swing lower bound
    static constexpr int kMaxAngleValue = 180;    // full-swing upper bound
    static constexpr int kCenterAngle   = 90;     // neutral / power-on pose
    static constexpr int kStep          = 1;      // degrees per rotate step
    static constexpr uint16_t kDefaultStepDelayMs = 15;  // fallback settle time

    // The servo only understands 0..180. A rig that keeps being driven in one
    // direction must therefore *wrap around* instead of latching at an end
    // stop: going past 180 returns to 0 and vice versa. That keeps the stick
    // responsive no matter how many full turns the operator has made.
    //
    // Wrapping is a property of a continuous (endless) axis. A mechanism with
    // real end stops must turn it off with setWrap(false) - see main-servo.cpp.
    static constexpr bool kDefaultWrap = true;

    // Build the axis and configure its travel limits. The servo is attached
    // later by begin(): limits are a property of the mechanism, the pin is a
    // property of the wiring, and setup() knows both.
    explicit Axis(int minAngle = kMinAngleValue,
                  int maxAngle = kMaxAngleValue,
                  int startAngle = kCenterAngle);

    // Configure the limits in place (keeps the current position, re-clamped).
    void setRange(int minAngle, int maxAngle);

    // Enable/disable wrap-around at the limits. When disabled the axis is a
    // limited axis: it stops at min_/max_ (mechanisms with end stops).
    void setWrap(bool wrap) { wrap_ = wrap; }
    bool wraps() const { return wrap_; }

    // Attach the servo to `pin` and drive it to the given pose. Must be called
    // before any other motion method.
    void begin(uint8_t pin, int startAngle = kCenterAngle);

    // Current angle in degrees (0..180).
    int  getPos() const { return pos_; }
    bool isMoving() const { return pos_ != target_; }

    // True when the axis can no longer move in the given direction: a limited
    // axis sitting on an end stop. Wrapping axes never report "stuck".
    bool blockedClockwise() const { return !wrap_ && pos_ >= max_; }
    bool blockedCounterClockwise() const { return !wrap_ && pos_ <= min_; }

    // Jump to an absolute angle (clamped to the configured limits).
    void setPos(int angle);

    // Smoothly walk to an absolute angle, one step per iteration, waiting
    // stepDelayMs after each step so the servo can physically get there.
    // Blocking - use rotate*() from a non-blocking loop() instead.
    void moveTo(int angle, uint16_t stepDelayMs = kDefaultStepDelayMs);

    // Advance exactly ONE step toward the upper/lower limit and wait
    // stepDelayMs. Used to drive the axis from a joystick: the caller decides
    // *whether* to step, the axis only knows *how*.
    //
    // Returns true when the axis actually moved. On a limited axis sitting on
    // its end stop this returns false, which lets the caller detect "stuck".
    bool rotateClockwise(uint16_t stepDelayMs = kDefaultStepDelayMs);
    bool rotateCounterClockwise(uint16_t stepDelayMs = kDefaultStepDelayMs);

    // Sweep the whole travel to calibrate the mechanism. Blocking.
    void sweepToMin();
    void sweepToMax();

    bool atMax() const { return pos_ >= max_; }
    bool atMin() const { return pos_ <= min_; }
    int  minAngle() const { return min_; }
    int  maxAngle() const { return max_; }

private:
    Servo servo_;
    int  pos_    = kCenterAngle;
    int  target_ = kCenterAngle;
    int  min_    = kMinAngleValue;
    int  max_    = kMaxAngleValue;
    bool attached_ = false;
    bool wrap_     = kDefaultWrap;

    // Apply one step of `delta` degrees, honouring wrap-around on a wrapping
    // axis and clamping on a limited one. Returns true when the axis moved.
    bool stepBy(int delta, uint16_t stepDelayMs);

    void writePos(int angle);
};

