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
    // no matter which public method asked for the move. The clamp is what stops
    // the servo being driven past its mechanical travel.
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

bool Axis::stepBy(int delta) {
    // Limited axis: walk toward the limit and stop dead on it. No wrap-around -
    // see the header for why an SG90 must never be told to jump across its arc.
    const int next = constrain(pos_ + delta, min_, max_);

    if (next == pos_) {
        return false;                 // on the end stop: nothing to do
    }

    writePos(next);
    return true;                      // never sleeps: the caller paces us
}

bool Axis::rotateClockwise() {
    return stepBy(kStep);
}

bool Axis::rotateCounterClockwise() {
    return stepBy(-kStep);
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
