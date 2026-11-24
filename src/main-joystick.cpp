#include <Arduino.h>

#include "Joystick/Joystick.h"
#include "Link/JoystickFrame.h"
#include "Link/LinkMaster.h"

// MASTER BOARD - the one with the joystick.
//
// Owns no protocol or motion code: it reads a Joystick, packages the reading
// into a JoystickFrame and hands it to a LinkMaster. Everything about "how the
// bits travel" lives in the Link layer, everything about the hardware in
// Joystick.
//
// Wiring
//   VRx        -> A0
//   VRy        -> A1
//   SW         -> D2   (INPUT_PULLUP, active low)
//   SDA/SCL    -> A4/A5 (same bus as the servo board)
//   D13        -> onboard LED, lit while the servo board acknowledges frames
namespace {

constexpr uint8_t  kPinJoyX        = A0;
constexpr uint8_t  kPinJoyY        = A1;
constexpr uint8_t  kPinJoyBtn      = 2;
constexpr uint8_t  kPinLinkLed     = 13;
constexpr uint8_t  kServoBoardAddr = 8;   // must match LinkSlave on the servo board
constexpr uint16_t kSendPeriodMs   = 15;  // ~66 Hz, matches the servo loop

// An idle stick jitters by a few counts. Treating that as "changed" would make
// the master transmit continuously, so a frame only counts as new once the
// reading has moved by at least this much (or the button changed).
constexpr int kChangeThreshold = 4;

// Safety net: even when nothing changes, re-send this often. If a single frame
// is ever lost, or the slave resets on its own, the link heals by itself
// instead of waiting for the operator to move the stick again.
constexpr uint16_t kKeepAliveMs = 200;

Joystick    joystick(kPinJoyX, kPinJoyY, kPinJoyBtn);
LinkMaster  link(kServoBoardAddr);

JoystickFrame lastSent;
bool          everSent   = false;   // the first frame is always sent
uint32_t      lastSendMs = 0;

}  // namespace

void setup() {
    joystick.begin();
    link.begin();

    pinMode(kPinLinkLed, OUTPUT);
    digitalWrite(kPinLinkLed, LOW);
}

void loop() {
    static uint32_t nextSendMs = 0;

    const uint32_t now = millis();
    if ((int32_t)(now - nextSendMs) < 0) {
        return;                        // not time to send yet
    }
    nextSendMs = now + kSendPeriodMs;

    JoystickFrame frame;
    // Send the RAW stick position, not the deadband-snapped one. The servo
    // board owns the dead zone (AxisDriver): it needs the true distance from
    // centre to pick a direction and a speed. Snapping here as well would mean
    // two dead zones fighting each other.
    frame.x   = joystick.rawX();
    frame.y   = joystick.rawY();
    frame.btn = joystick.buttonDown(now);
    frame.clamp();

    const bool moved = !everSent ||
                       abs(frame.x - lastSent.x) >= kChangeThreshold ||
                       abs(frame.y - lastSent.y) >= kChangeThreshold ||
                       frame.btn != lastSent.btn;
    const bool keepAlive = (uint32_t)(now - lastSendMs) >= kKeepAliveMs;

    // An idle stick must not hammer the bus, but a lost link must recover on
    // its own - hence the keep-alive.
    if (!moved && !keepAlive && link.isLinkUp()) {
        return;
    }

    if (link.send(frame)) {
        lastSent   = frame;
        everSent   = true;
        lastSendMs = now;
        digitalWrite(kPinLinkLed, HIGH);   // slave acknowledged us
    } else {
        // Keep everSent = false so the next pass retries the same reading even
        // if the stick has not moved.
        everSent = false;
        digitalWrite(kPinLinkLed, LOW);
    }
}

