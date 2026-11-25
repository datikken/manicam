#pragma once

// Host-side stand-in for <Servo.h>.
//
// A real Servo drives a PWM pin. Here it records what it was told so a test can
// assert on the *commands*, which is exactly what Axis is responsible for: it
// translates positions into write() calls and must never exceed its limits.
//
// Every attach/write is logged in order, because "did the axis ever command an
// out-of-range angle on the way to a clamp?" is a question worth answering -
// that is the bug that stresses an SG90.

#include <cstdint>
#include <vector>

namespace servo_shim {

struct ServoState {
    int          lastWritten = -1;   // -1 = never written
    int          attachPin   = -1;
    bool         attached    = false;
    std::vector<int> history;        // every write(), in order
    int          writeCount  = 0;

    void reset() {
        lastWritten = -1;
        attachPin   = -1;
        attached    = false;
        history.clear();
        writeCount  = 0;
    }
};

// One shim state per Servo instance is impossible without a per-object hook, so
// tests share a single state and reset it in setUp(). The application owns at
// most two servos, and each test exercises one axis at a time.
inline ServoState& state() {
    static ServoState s;
    return s;
}

}  // namespace servo_shim

class Servo {
public:
    void attach(int pin) {
        servo_shim::state().attachPin = pin;
        servo_shim::state().attached  = true;
    }

    void detach() {
        servo_shim::state().attached = false;
    }

    bool attached() const { return servo_shim::state().attached; }

    void write(int value) {
        servo_shim::state().lastWritten = value;
        servo_shim::state().history.push_back(value);
        servo_shim::state().writeCount++;
    }

    int read() const { return servo_shim::state().lastWritten; }
};
