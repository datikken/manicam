#include <Arduino.h>

#include "Axis/Axis.h"
#include "AxisDriver/AxisDriver.h"
#include "Link/JoystickFrame.h"
#include "Link/LinkSlave.h"
#include "PositionStore/PositionStore.h"

// SLAVE BOARD - the one with the servos.
//
// Wires four objects together and nothing more:
//   LinkSlave      receives frames from the joystick board over I2C
//   Axis x2        own the servos, their limits and their current angle
//   AxisDriver x2  convert a stick position into one step of motion
//   PositionStore  saves the pose and restores it after a power cycle
//
// Wiring
//   Servo X  -> D9
//   Servo Y  -> D10
//   LED      -> D4   (lit while the button on the joystick is pressed)
//   SDA/SCL  -> A4/A5
namespace {

constexpr uint8_t  kOwnAddr   = 8;    // must match LinkMaster on the joystick board
constexpr uint8_t  kPinServoX = 9;
constexpr uint8_t  kPinServoY = 10;
constexpr uint8_t  kPinLed    = 4;

// Travel limits per axis, kept INSIDE the SG90's mechanical end stops.
//
// An SG90 can be driven to 0 and 180 electrically, but its plastic gears and
// internal stop cannot take being parked there under power: it buzzes, draws
// stall current and overheats. Leaving a few degrees of margin is what keeps a
// micro servo alive. Tighten further if the mechanism has its own hard stops.
constexpr int kAxisXMin = 10;
constexpr int kAxisXMax = 170;
constexpr int kAxisYMin = 10;
constexpr int kAxisYMax = 170;

// The rig is driven by standard positional SG90 servos: about 180 degrees of
// travel each, no continuous rotation. The axis therefore stops at its limits
// instead of wrapping - see Axis.h for the full reasoning.
constexpr bool kAxisIsContinuous = false;

Axis        axisX(kAxisXMin, kAxisXMax);
Axis        axisY(kAxisYMin, kAxisYMax);
AxisDriver  driverX(axisX);
AxisDriver  driverY(axisY);
PositionStore positions(axisX, axisY);
LinkSlave   link(kOwnAddr);
}  // namespace

void setup() {
    axisX.begin(kPinServoX);
    axisY.begin(kPinServoY);

  pinMode(kPinLed, OUTPUT);
  digitalWrite(kPinLed, LOW);

  // Bring the rig back to the pose it had before the power was cut. Must run
  // before the link starts, otherwise the first incoming frame could overwrite
  // the restored pose before it is ever written to EEPROM.
  positions.restore();

    link.begin();
}

void loop() {
  JoystickFrame frame;

    // take() reports whether this is a fresh frame. Until the very first frame
    // arrives there is nothing to act on: without this check a default-built
    // frame (x = y = 0) would look like "stick slammed to minimum" and drive
    // both axes off in one direction.
    if (!link.take(frame) && !link.hasData()) {
        positions.update();
        return;
    }

    // When no new frame arrived we keep working with the last known stick
    // position, so a slower sender cannot stall the servos mid-motion.
    driverX.update(frame.x);
    driverY.update(frame.y);

  digitalWrite(kPinLed, frame.btn ? HIGH : LOW);

  positions.update();      // persists the pose once the axes settle
}


