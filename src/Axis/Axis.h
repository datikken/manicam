#pragma once

#include <Arduino.h>
#include <Servo.h>

// One servo-driven axis of the camera rig.
//
// The class owns the Servo object, the current angle and the travel limits.
// Every write goes through clamp(), so "position bookkeeping" and "physical
// output" can never disagree - the rest of the firmware may ask for any angle
// it likes and the stored `pos` always matches what the servo was told.
//
// Angles are degrees in [kMinAngleValue, kMaxAngleValue] - the range every
// hobby servo understands. Real mechanisms rarely use the full swing, which is
// why the limits are configurable.
class Axis {
public:
    static constexpr int kMinAngleValue = 0;      // full-swing lower bound
    static constexpr int kMaxAngleValue = 180;    // full-swing upper bound
    static constexpr int kCenterAngle   = 90;     // neutral / power-on pose
    static constexpr int kStep          = 1;      // degrees per rotate step
    static constexpr uint16_t kDefaultStepDelayMs = 15;  // fallback settle time

    // Build the axis and configure its travel limits. The servo is attached
    // later by begin(): limits are a property of the mechanism, the pin is a
    // property of the wiring, and setup() knows both.
    explicit Axis(int minAngle = kMinAngleValue,
                  int maxAngle = kMaxAngleValue,
                  int startAngle = kCenterAngle);

    // Configure the limits in place (keeps the current position, re-clamped).
    void setRange(int minAngle, int maxAngle);

    // Attach the servo to `pin` and drive it to the given pose. Must be called
    // before any other motion method.
    void begin(uint8_t pin, int startAngle = kCenterAngle);

    // Current angle in degrees (0..180).
    int  getPos() const { return pos_; }
    bool isMoving() const { return pos_ != target_; }

    // Jump to an absolute angle (clamped to the configured limits).
    void setPos(int angle);

    // Smoothly walk to an absolute angle, one step per iteration, waiting
    // stepDelayMs after each step so the servo can physically get there.
    // Blocking - use rotate*() from a non-blocking loop() instead.
    void moveTo(int angle, uint16_t stepDelayMs = kDefaultStepDelayMs);

    // Advance exactly ONE step toward the upper/lower limit and wait
    // stepDelayMs. Used to drive the axis from a joystick: the caller decides
    // *whether* to step, the axis only knows *how*.
    void rotateClockwise(uint16_t stepDelayMs = kDefaultStepDelayMs);
    void rotateCounterClockwise(uint16_t stepDelayMs = kDefaultStepDelayMs);

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

    void writePos(int angle);
};

