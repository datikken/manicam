#include <Arduino.h>

#include "Axis/Axis.h"
#include "AxisDriver/AxisDriver.h"
#include "Joystick/Joystick.h"
#include "Link/JoystickFrame.h"
#include "Link/LinkMaster.h"

// MASTER BOARD - the one with the joystick.
//
// Owns no protocol or math code: it reads a Joystick, packages the reading into
// a JoystickFrame and hands it to a LinkMaster. Everything about "how the bits
// travel" lives in the Link layer, everything about the hardware in Joystick.
//
// Wiring
//   VRx        -> A0
//   VRy        -> A1
//   SW         -> D2   (INPUT_PULLUP, active low)
//   SDA/SCL    -> A4/A5 (same bus as the servo board)
//   D13        -> onboard LED, lit while the servo board acknowledges frames
namespace {

constexpr uint8_t  kPinJoyX     = A0;
constexpr uint8_t  kPinJoyY     = A1;
constexpr uint8_t  kPinJoyBtn   = 2;
constexpr uint8_t  kPinLinkLed  = 13;
constexpr uint8_t  kServoBoardAddr = 8;   // must match LinkSlave on the servo board
constexpr uint16_t kSendPeriodMs   = 15;  // ~66 Hz, matches the servo loop
Joystick    joystick(kPinJoyX, kPinJoyY, kPinJoyBtn);
LinkMaster  link(kServoBoardAddr);

JoystickFrame lastSent;
bool          everSent = false;   // the first frame is always sent
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
  frame.x   = joystick.x();
  frame.y   = joystick.y();
  frame.btn = joystick.buttonDown(now);

  // Only talk when something changed: an idle stick must not hammer the bus.
  // When the link was lost, resend even identical data so the slave resyncs.
  const bool changed = !everSent || frame.x != lastSent.x ||
                       frame.y != lastSent.y || frame.btn != lastSent.btn;
  if (!changed && link.isLinkUp()) {
        return;
    }

    if (link.send(frame)) {
        lastSent = frame;
    everSent = true;
    digitalWrite(kPinLinkLed, HIGH);   // slave acknowledged us
    } else {
    everSent = false;                  // retry the same frame next pass
        digitalWrite(kPinLinkLed, LOW);
    }
}

