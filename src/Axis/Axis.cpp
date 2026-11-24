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

bool Axis::stepBy(int delta, uint16_t stepDelayMs) {
    int next = pos_ + delta;

    if (wrap_) {
        // Continuous axis: wrap around the full swing instead of latching at
        // an end stop. This is what keeps the stick responsive after any
        // number of full turns.
        const int span = max_ - min_ + 1;
        if (span > 0) {
            next = min_ + (((next - min_) % span) + span) % span;
        }
    } else {
        // Limited axis: stop dead on the end stop.
        next = constrain(next, min_, max_);
    }

    if (next == pos_) {
        return false;                 // at the end stop: nothing to do
    }

    writePos(next);
    delay(stepDelayMs);               // let the servo physically reach this step
    return true;
}

bool Axis::rotateClockwise(uint16_t stepDelayMs) {
    return stepBy(kStep, stepDelayMs);
}

bool Axis::rotateCounterClockwise(uint16_t stepDelayMs) {
    return stepBy(-kStep, stepDelayMs);
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
