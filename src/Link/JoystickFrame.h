#pragma once

#include <Arduino.h>

// Wire-format contract shared by BOTH boards.
//
// The joystick board (master) and the servo board (slave) must agree on it,
// so it lives in exactly one place and is included by both firmwares.
//
// Frame layout (5 bytes, big-endian):
//   [0] xH   [1] xL   [2] yH   [3] yL   [4] btn
//
//   x, y  - raw joystick readings, 0..1023 (10-bit ADC)
//   btn   - 0 = released, 1 = pressed
//
// The frame is deliberately "dumb": it carries the raw stick position, not an
// angle. Turning a stick position into a servo angle is the servo board's job.
struct JoystickFrame {
    static constexpr uint8_t kSize = 5;     // bytes on the wire
    static constexpr int kAdcMin = 0;       // 10-bit ADC range
    static constexpr int kAdcMax = 1023;

    int x = 0;                              // 0..1023
    int y = 0;                              // 0..1023
    bool btn = false;

    // Serialise this frame into out[0..kSize-1]. Returns kSize.
    uint8_t encode(uint8_t* out) const;

    // Parse a frame from in[0..kSize-1]. Returns true when the input is a
    // well-formed frame; on failure the frame is left untouched.
    bool decode(const uint8_t* in);

    // Clamp the axes into the valid ADC range.
    void clamp();
};
