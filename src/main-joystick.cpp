#include <Arduino.h>
#include "Joystick/Joystick.h"
#include "Link/JoystickFrame.h"
#include "Link/LinkMaster.h"

// ---------------------------------------------------------------------------
// MASTER BOARD - the one with the joystick.
//
// Wiring (Arduino Nano):
//   VRx      -> A0          VRy     -> A1
//   SW       -> D2  (INPUT_PULLUP, pressed = GND)
//   SDA/SCL  -> A4 / A5     link LED -> D13 (on-board LED)
//
// This file contains no protocol logic: it only *wires objects together*.
//   Joystick      - reads and filters the sticks/button
//   JoystickFrame - the wire format both boards share
//   LinkMaster    - pushes a frame down the I2C bus
//
// Behaviour: sample at kSendPeriodMs, skip frames when nothing changed (so the
// bus stays quiet), and light the LED while the slave acknowledges.
// ---------------------------------------------------------------------------

namespace {

constexpr uint8_t kPinJoyX   = A0;
constexpr uint8_t kPinJoyY   = A1;
constexpr uint8_t kPinButton = 2;
constexpr uint8_t kPinLink   = LED_BUILTIN;

constexpr uint8_t  kSlaveAddress = 8;    // must match LinkSlave on the servo board
constexpr uint16_t kSendPeriodMs = 15;   // ~66 Hz - matches the servo refresh rate

Joystick   g_joystick(kPinJoyX, kPinJoyY, kPinButton);
LinkMaster g_link(kSlaveAddress);

// Last frame the slave acknowledged. A frame is only re-sent when it differs,
// so an untouched stick costs no bus traffic; the first frame is always sent so
// the servo board learns about us.
JoystickFrame g_lastSent;
bool          g_hasSent     = false;
uint32_t      g_lastSendMs  = 0;

// Read a live frame out of the joystick object.
void readFrame(JoystickFrame& frame, uint32_t now) {
    frame.x   = g_joystick.x();
    frame.y   = g_joystick.y();
    frame.btn = g_joystick.buttonDown(now);
    frame.clamp();
}

bool sameAsLast(const JoystickFrame& frame) {
    return g_hasSent &&
           frame.x == g_lastSent.x &&
           frame.y == g_lastSent.y &&
           frame.btn == g_lastSent.btn;
}

}  // namespace
void setup() {
    g_joystick.begin();
    g_link.begin();

    pinMode(kPinLink, OUTPUT);
    digitalWrite(kPinLink, LOW);
}

void loop() {
    const uint32_t now = millis();
    if (now - g_lastSendMs < kSendPeriodMs) {
        return;
    }
    g_lastSendMs = now;

    JoystickFrame frame;
    readFrame(frame, now);

    if (sameAsLast(frame)) {
        return;                       // nothing changed: keep the bus quiet
    }

    if (g_link.send(frame)) {
        g_lastSent = frame;           // acknowledged: remember it
        g_hasSent  = true;
        digitalWrite(kPinLink, HIGH);
    } else {
        // Slave did not answer. Forget the cached frame so the next pass
        // retries with fresh data instead of staying silent forever.
        g_hasSent = false;
        digitalWrite(kPinLink, LOW);
    }
}

