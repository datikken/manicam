#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Link/JoystickFrame.h"

// Master side of the joystick -> servo link.
//
// Owns nothing but the bus state: it writes frames to the slave address and
// reports whether the last transmission was acknowledged. Timing, change
// detection and the decision "should I send?" live in the caller, so this
// class stays trivially testable and reusable.
class LinkMaster {
public:
    explicit LinkMaster(uint8_t slaveAddress, TwoWire& bus = Wire);

    // Bring up the bus as a master. Safe to call once from setup().
    void begin();

    // Send one frame. Returns true when the slave acknowledged it (link is up).
    bool send(const JoystickFrame& frame);

    // Result of the most recent send(): true = last frame was acknowledged.
    bool isLinkUp() const { return linkUp_; }

    uint8_t slaveAddress() const { return slaveAddr_; }

private:
    uint8_t slaveAddr_;
    TwoWire& bus_;
    bool linkUp_ = false;
};
