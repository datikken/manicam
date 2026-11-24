#include "Axis/Axis.h"

Axis::Axis(int minAngle, int maxAngle, int startAngle) {
    setRange(minAngle, maxAngle);
    pos_    = constrain(startAngle, min_, max_);
    target_ = pos_;
}

void Axis::setRange(int minAngle, int maxAngle) {
    // Tolerate a swapped pair: the limits describe the mechanism, not the
    // order in which a caller happens to spell them out.
    min_ = constrain(minAngle, kMinAngleValue, kMaxAngleValue);
    max_ = constrain(maxAngle, kMinAngleValue, kMaxAngleValue);
    if (min_ > max_) {
        const int tmp = min_;
        min_ = max_;
        max_ = tmp;
    }
    pos_    = constrain(pos_, min_, max_);
    target_ = constrain(target_, min_, max_);
    if (attached_) {
        servo_.write(pos_);
    }
}

void Axis::begin(uint8_t pin, int startAngle) {
    servo_.attach(pin);
    attached_ = true;
    pos_    = constrain(startAngle, min_, max_);
    target_ = pos_;
    servo_.write(pos_);
}

void Axis::writePos(int angle) {
    // Single funnel for every servo write: limits and bookkeeping stay in sync
    // no matter which public method asked for the move.
    pos_ = constrain(angle, min_, max_);
    if (attached_) {
        servo_.write(pos_);
    }
}

void Axis::setPos(int angle) {
    writePos(angle);
    target_ = pos_;
}

void Axis::moveTo(int angle, uint16_t stepDelayMs) {
    target_ = constrain(angle, min_, max_);
    while (pos_ != target_) {
        if (target_ > pos_) {
            writePos(pos_ + kStep);   // writePos() clamps: never overshoots
        } else {
            writePos(pos_ - kStep);
        }
        delay(stepDelayMs);
    }
}

void Axis::rotateClockwise(uint16_t stepDelayMs) {
    if (pos_ >= max_) {
        return;                       // already at the upper limit
    }
    writePos(pos_ + kStep);
    delay(stepDelayMs);               // let the servo physically reach this step
}

void Axis::rotateCounterClockwise(uint16_t stepDelayMs) {
    if (pos_ <= min_) {
        return;                       // already at the lower limit
    }
    writePos(pos_ - kStep);
    delay(stepDelayMs);
}

void Axis::sweepToMax() {
    for (int a = min_; a <= max_; a += kStep) {
        writePos(a);
        delay(kDefaultStepDelayMs);
    }
    writePos(max_);
}

void Axis::sweepToMin() {
    for (int a = max_; a >= min_; a -= kStep) {
        writePos(a);
        delay(kDefaultStepDelayMs);
    }
    writePos(min_);
}
