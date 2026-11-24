#include <Arduino.h>
#include "Axis/Axis.h"
#include "AxisDriver/AxisDriver.h"
#include "Link/JoystickFrame.h"
#include "Link/LinkSlave.h"
#include "PositionStore/PositionStore.h"

// ---------------------------------------------------------------------------
// SLAVE BOARD - the one with the two servos.
//
// Wiring (Arduino Nano):
//   servo X signal -> D9        servo Y signal -> D10
//   status LED     -> D4        SDA/SCL         -> A4 / A5
//   (power the servos from a separate 5 V supply with a shared ground -
//    the Nano's regulator cannot feed two servos)
//
// Again, this file only wires objects together:
//   LinkSlave     - receives frames, hands over a consistent snapshot
//   Axis          - owns a Servo + its angle/limits
//   AxisDriver    - turns a stick reading into axis motion
//   PositionStore - saves the pose to EEPROM and restores it after power-up
//
// The joystick moves each axis *relatively*: holding the stick drives the servo
// at a speed proportional to how far it is pushed, and releasing it holds the
// pose. That is what lets the saved position survive a power cycle - an
// absolute mapping would snap the servo back to the stick's resting point.
// ---------------------------------------------------------------------------
namespace {

constexpr uint8_t kPinServoX = 9;
constexpr uint8_t kPinServoY = 10;
constexpr uint8_t kPinLed    = 4;

constexpr uint8_t kOwnAddress = 8;        // must match LinkMaster on the joystick board

// Travel limits. Narrow these to the real mechanism's safe swing; full 0..180
// is only the servo's own range, not necessarily yours.
constexpr int kMinAngle = 10;
constexpr int kMaxAngle = 170;

constexpr uint32_t kLedBlinkMs = 500;     // "no signal yet" heartbeat

// Objects. Axis first: the drivers and the store hold references to them.
Axis          g_axisX(kMinAngle, kMaxAngle, Axis::kCenterAngle);
Axis          g_axisY(kMinAngle, kMaxAngle, Axis::kCenterAngle);
AxisDriver    g_driverX(g_axisX);
AxisDriver    g_driverY(g_axisY);
LinkSlave     g_link(kOwnAddress);
PositionStore g_store(g_axisX, g_axisY);

JoystickFrame g_frame;                    // latest snapshot from the master
bool          g_haveSignal = false;
uint32_t      g_lastBlinkMs = 0;
bool          g_ledOn = false;

void updateSignalLed(uint32_t now) {
    if (g_haveSignal) {
        digitalWrite(kPinLed, HIGH);      // solid while the master is talking
        g_ledOn = true;
        return;
    }
    if (now - g_lastBlinkMs >= kLedBlinkMs) {
        g_lastBlinkMs = now;
        g_ledOn = !g_ledOn;
        digitalWrite(kPinLed, g_ledOn);
    }
}

}  // namespace

void setup() {
    pinMode(kPinLed, OUTPUT);
    digitalWrite(kPinLed, LOW);

    // Attach the servos at the neutral pose first, then let the store move them
    // to the pose they had before the power was cut.
    g_axisX.begin(kPinServoX);
    g_axisY.begin(kPinServoY);
    g_store.restore();

    g_link.begin();
}

void loop() {
    const uint32_t now = millis();

    // Pick up a freshly received frame, if the master sent one.
    if (g_link.take(g_frame)) {
        g_haveSignal = true;
    }
    // Drive the axes: one step per pass, speed taken from the stick position.
    g_driverX.update(g_frame.x);
    g_driverY.update(g_frame.y);

    // Persist the pose once the axes stop moving.
    g_store.update();

    updateSignalLed(now);
}

