#pragma once

// Helpers shared by every test file: resetting the fake board, advancing the
// virtual clock, and driving pins. Keeping them here means each test file does
// not repeat the same four lines of setup.

#include "Arduino.h"
#include "EEPROM.h"
#include "Servo.h"
#include "Wire.h"

namespace fixture {

// Put the fake world back to a known state. Called at the start of every test.
inline void reset() {
    arduino_shim::board().reset();
    servo_shim::state().reset();
    eeprom_shim::state().wipe();
    wire_shim::state().reset();
}

// Advance the virtual clock. Nothing in the application should depend on real
// elapsed time, so tests move time explicitly and stay instant.
inline void advanceMs(uint32_t ms) {
    arduino_shim::board().nowMs += ms;
}

inline uint32_t nowMs() { return arduino_shim::board().nowMs; }

// Set the level a digital pin will report on the next digitalRead().
inline void setPin(uint8_t pin, int level) {
    if (pin < 32) {
        arduino_shim::board().pinLevel[pin] = level;
    }
}

inline void setAnalog(uint8_t pin, int value) {
    if (pin < 32) {
        arduino_shim::board().analogValue[pin] = value;
    }
}

// The most recent angle the servo was told to go to.
inline int servoAngle() { return servo_shim::state().lastWritten; }

// Every angle the servo was ever told to go to, in order.
inline const std::vector<int>& servoHistory() {
    return servo_shim::state().history;
}

// True if any commanded angle ever fell outside [lo, hi]. This is the check
// that catches the class of bug that destroys an SG90.
inline bool servoEverOutside(int lo, int hi) {
    for (int v : servo_shim::state().history) {
        if (v < lo || v > hi) {
            return true;
        }
    }
    return false;
}

}  // namespace fixture
