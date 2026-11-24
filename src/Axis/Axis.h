#pragma once

#include <Arduino.h>
#include <Servo.h>

// One servo-driven axis of the camera rig.
//
// The servo is a Tower Pro SG90: a STANDARD POSITIONAL micro servo. It has
// about 180 degrees of mechanical travel and no feedback, so:
//
//   * an angle of 0..180 is a POSITION, not a speed;
//   * it physically CANNOT complete a full rotation;
//   * asking for a position far across its arc makes it swing there as fast as
//     it can - it is not a "rotate" command.
//
// Therefore the axis is deliberately LIMITED: it clamps to [min_, max_] and
// stops there. There is no wrap-around. Wrapping the angle 180 -> 0 would tell
// the servo to slam its arm backwards across the whole arc, which stalls the
// motor, overheats the SG90's tiny driver and makes it stop responding - the
// "works for a few turns, then dies" failure.
//
// An SG90 is also a light-duty part: parking it against a mechanical end stop
// under power makes it buzz and heat up. main-servo.cpp therefore sets limits
// slightly inside the mechanical extremes.
class Axis {
public:
    static constexpr int kMinAngleValue = 0;      // full-swing lower bound
    static constexpr int kMaxAngleValue = 180;    // full-swing upper bound
    static constexpr int kCenterAngle   = 90;     // neutral / power-on pose
    static constexpr int kStep          = 1;      // degrees per rotate step
    static constexpr uint16_t kDefaultStepDelayMs = 20;  // settle time, SG90

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

    // True when the axis sits on a travel limit and cannot move further in that
    // direction. Lets the caller tell "at the end" from "still moving".
    bool blockedClockwise() const { return pos_ >= max_; }
    bool blockedCounterClockwise() const { return pos_ <= min_; }
    bool atEnd() const { return blockedClockwise() || blockedCounterClockwise(); }

    // Jump to an absolute angle (clamped to the configured limits).
    void setPos(int angle);

    // Smoothly walk to an absolute angle, one step per iteration, waiting
    // stepDelayMs after each step so the servo can physically get there.
    // Blocking - use rotate*() from a non-blocking loop() instead.
    void moveTo(int angle, uint16_t stepDelayMs = kDefaultStepDelayMs);

    // Advance exactly ONE degree toward the upper/lower limit, stopping at it.
    // Used to drive the axis from a joystick: the caller decides *whether* to
    // step, the axis only knows *how*.
    //
    // These do NOT sleep. Rate limiting is the caller's job (see
    // AxisDriver::update), so the servo board's loop() keeps servicing I2C
    // while the axis moves.
    //
    // Returns true when the axis actually moved; false means it is already on
    // the end stop.
    bool rotateClockwise();
    bool rotateCounterClockwise();

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

    // Move by `delta` degrees, clamped to the limits. Returns true on movement.
    // Never blocks.
    bool stepBy(int delta);

    void writePos(int angle);
};
