#pragma once

#include <Arduino.h>

// Analogue thumbstick plus its push-button, as found on the joystick board.
//
// Responsibilities:
//   * own the pin numbers (A0/A1 for the axes, a digital pin for the switch);
//   * return raw 10-bit readings, and a centre-suppressed ("deadband") reading
//     so a stick resting slightly off-centre reads as exactly kCenter;
//   * debounce the button on a time basis, with no external library.
//
// The class does not know about servos, I2C or the wire format: it is a pure
// input device. Anything protocol-related lives in the Link layer, so the same
// Joystick can feed a local sketch or a remote board.
class Joystick {
public:
    static constexpr int kCenter = 512;     // ADC value of a resting stick
    static constexpr int kAdcMin = 0;       // 10-bit ADC range
    static constexpr int kAdcMax = 1023;
    static constexpr uint16_t kDefaultDeadband = 12;   // +/- around kCenter
    static constexpr uint16_t kDefaultDebounceMs = 20;

    Joystick(uint8_t pinX, uint8_t pinY, uint8_t pinButton,
             uint16_t deadband = kDefaultDeadband,
             uint16_t debounceMs = kDefaultDebounceMs);

    // Configure the pins. Call once from setup().
    void begin();

    int rawX() const { return analogRead(pinX_); }
    int rawY() const { return analogRead(pinY_); }

    // Readings with the deadband applied: anything within +/- deadband of the
    // centre is reported as exactly kCenter, which lets the receiver treat
    // "centre" as "no motion" without extra filtering.
    int x() const { return applyDeadband(rawX()); }
    int y() const { return applyDeadband(rawY()); }

    // Debounced button state: true while pressed. Pass millis().
    bool buttonDown(uint32_t now);

    // Print the current readings to Serial (debug helper).
    void debugPrint() const;

    uint16_t deadband() const { return deadband_; }
    void setDeadband(uint16_t deadband) { deadband_ = deadband; }

private:
    int applyDeadband(int value) const;

    uint8_t  pinX_;
    uint8_t  pinY_;
    uint8_t  pinButton_;
    uint16_t deadband_;
    uint16_t debounceMs_;

    // Button debounce state (all in millis() time).
    bool     btnStable_  = false;
    bool     btnRaw_     = false;
    uint32_t btnChanged_ = 0;
};

