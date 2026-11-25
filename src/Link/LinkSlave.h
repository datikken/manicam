#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Link/JoystickFrame.h"

// Slave side of the joystick -> servo link.
//
// The whole "ISR writes, loop() reads" problem is contained here:
//
//  * attach() registers a private static trampoline as Wire.onReceive. The
//    trampoline needs to get back to the object, and on AVR the I2C driver
//    gives it no user context, so the instance registers itself as "the"
//    receiver. Only one LinkSlave may exist per program - that is exactly how
//    many servo boards there are, so the constraint is free.
//
//  * The received frame is double-buffered and published under a short
//    interrupt-off critical section: on an 8-bit AVR plain int writes take
//    several instructions, and loop() could observe a torn (x, y) pair.
//
//  * take() hands loop() a consistent snapshot and tells it whether that
//    snapshot is newer than the one handed out last time.
class LinkSlave {
public:
    explicit LinkSlave(uint8_t ownAddress, TwoWire& bus = Wire);

    // Join the bus under ownAddress and start accepting frames.
    void begin();

    // Copy the latest received frame into `out`.
    // Returns true when at least one new, valid frame arrived since the last
    // take() call; false when `out` is merely a repeat of older data.
    bool take(JoystickFrame& out);

    // True from the first valid frame onwards: the master has talked to us.
    bool hasData() const { return hasData_; }

    uint8_t ownAddress() const { return ownAddr_; }

    // Diagnostic counters - used by a self-test build to answer "are frames
    // arriving at all, and are they being rejected?" without a logic analyser.
    //   accepted  - frames that passed size + range checks
    //   rejected  - frames that arrived but failed a check
    //   lastCount - byte count reported by the ISR for the latest frame
    uint16_t acceptedCount() const { return accepted_; }
    uint16_t rejectedCount() const { return rejected_; }
    int      lastByteCount() const { return lastCount_; }
    void     resetStats() { accepted_ = 0; rejected_ = 0; lastCount_ = -1; }

private:
    // The host test suite simulates the I2C interrupt by calling the handler
    // directly; on hardware only the trampoline below ever does. Friendship
    // keeps the public surface unchanged.
    friend struct LinkSlaveTestHook;

    static void onReceiveTrampoline(int count);

    void handleReceive(int count);
    void publish(const JoystickFrame& frame);

    uint8_t ownAddr_;
    TwoWire& bus_;

    // Written from the ISR, read from loop() -> must be volatile and published
    // under a critical section.
    volatile int  pendingX_   = 0;
    volatile int  pendingY_   = 0;
    volatile bool pendingBtn_ = false;
    volatile bool pendingNew_ = false;   // a fresh frame is waiting
    volatile bool hasData_    = false;   // a frame has ever been received

    volatile uint16_t accepted_ = 0;
    volatile uint16_t rejected_ = 0;
    volatile int      lastCount_ = -1;
};


