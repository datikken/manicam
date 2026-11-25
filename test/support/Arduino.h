#pragma once

// Host-side stand-in for <Arduino.h>.
//
// Unit tests run on the development machine, not on the Nano, so the handful of
// Arduino calls the application uses are re-implemented here against a fake
// world that the tests can drive:
//
//   * millis()          - a virtual clock the test advances by hand
//   * digitalRead()     - returns whatever the test last told it to
//   * pinMode/write     - recorded, so tests can assert on them
//   * constrain/min/max - same behaviour as the Arduino macros
//
// Everything is deterministic: no real time, no real pins, no hardware. That is
// what makes the tests meaningful - a test that depends on the wall clock or on
// a servo would be flaky instead of proving a rule.

#include <cstdint>
#include <cmath>
#include <algorithm>

// Arduino type aliases the sources rely on.
using byte = uint8_t;

// --- Arduino constants ---------------------------------------------------

#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif

#ifndef INPUT
#define INPUT 0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP 2
#endif

// Analog pin names. On AVR these are just numbers above the digital pins; any
// distinct values work here because the shim indexes arrays of 32 entries.
#ifndef A0
#define A0 14
#endif
#ifndef A1
#define A1 15
#endif

// --- virtual clock -------------------------------------------------------

namespace arduino_shim {

// Global mutable state for the fake board. A single instance keeps the shim
// headers header-only and free of link-time surprises.
struct BoardState {
    uint32_t nowMs = 0;

    // Pin levels as last set by the test. Default HIGH so an active-low button
    // reads as "not pressed" until a test says otherwise.
    int pinLevel[32];

    // Values returned by analogRead(), indexed by pin number.
    int analogValue[32];

    // How many times each pin was written, and its last written level.
    int  digitalWrites[32];
    int  lastDigitalWrite[32];

    BoardState() {
        for (int i = 0; i < 32; ++i) {
            pinLevel[i]        = HIGH;
            analogValue[i]     = 512;
            digitalWrites[i]   = 0;
            lastDigitalWrite[i] = LOW;
        }
    }

    void reset() {
        nowMs = 0;
        for (int i = 0; i < 32; ++i) {
            pinLevel[i]         = HIGH;
            analogValue[i]      = 512;
            digitalWrites[i]    = 0;
            lastDigitalWrite[i] = LOW;
        }
    }
};

// Function-local static: one instance per program, no ODR problems.
inline BoardState& board() {
    static BoardState state;
    return state;
}

}  // namespace arduino_shim

// --- Arduino functions ---------------------------------------------------

inline uint32_t millis() { return arduino_shim::board().nowMs; }

inline uint32_t micros() { return arduino_shim::board().nowMs * 1000u; }

inline void delay(uint32_t ms) { arduino_shim::board().nowMs += ms; }

inline void pinMode(uint8_t, uint8_t) {}

inline void digitalWrite(uint8_t pin, uint8_t value) {
    if (pin < 32) {
        arduino_shim::board().digitalWrites[pin]++;
        arduino_shim::board().lastDigitalWrite[pin] = (int)value;
    }
}

inline int digitalRead(uint8_t pin) {
    return (pin < 32) ? arduino_shim::board().pinLevel[pin] : HIGH;
}

inline int analogRead(uint8_t pin) {
    return (pin < 32) ? arduino_shim::board().analogValue[pin] : 512;
}

// Arduino's constrain() is a macro; a template is safer and behaves the same
// for the integer types the application uses.
template <typename T>
inline T constrain(T value, T low, T high) {
    return (value < low) ? low : ((value > high) ? high : value);
}

// min()/max() come from <algorithm> via std:: but Arduino code calls them
// unqualified; pull them into the global namespace explicitly.
using std::max;
using std::min;
using std::abs;

// Serial is only used for debug printing, which tests do not exercise. A stub
// keeps debugPrint() compiling without pulling in an I/O layer.
class FakeSerial {
public:
    void begin(unsigned long) {}
    template <typename T> void print(const T&) {}
    template <typename T> void println(const T&) {}
    void println() {}
};

// Function-local static so every translation unit sees the same object.
inline FakeSerial& serialInstance() {
    static FakeSerial serial;
    return serial;
}

#define Serial serialInstance()

// SREG and cli() are used by LinkSlave for its critical sections. On the host
// there are no interrupts to disable, so they are no-ops that preserve the
// save/restore pattern.
// SREG is a register the sources save and restore around a critical section.
// On the host it is a plain variable, so the save/restore pattern compiles and
// keeps its shape even though there is nothing to disable.
inline uint8_t& sregValue() {
    static uint8_t sreg = 0;
    return sreg;
}
#define SREG sregValue()
inline void cli() {}
