#pragma once

#include <Arduino.h>
#include "Axis/Axis.h"

// Persists the servo positions so the rig comes back to the same pose after a
// power cut or a reset.
//
// An Arduino Nano has no filesystem; the 1 KiB onboard EEPROM survives power
// cycles, so that is the mechanism. Layout used here:
//
//   byte 0     magic marker (kMagic)  - "this EEPROM holds a valid pose"
//   byte 1     version (kVersion)     - lets a future format change be detected
//   byte 2..3  X angle (uint16)       - little-endian, 0..180
//   byte 4..5  Y angle (uint16)       - little-endian, 0..180
//   byte 6     checksum of bytes 1..5 - rejects a half-written / corrupt record
//
// EEPROM cells are rated for ~100k writes, and every EEPROM.update() of a
// changed byte blocks the CPU for ~3.3 ms - long enough to miss incoming I2C
// frames. Saving is therefore doubly lazy:
//
//   1. the debounce timer restarts every time the pose changes, so a save can
//      never fire while an axis is still moving;
//   2. the record is written once the axes have been still for
//      kSaveDebounceMs, and skipped entirely if that exact pose is already
//      stored.
//
// Without (1) a fast stress test wrote EEPROM continuously, and the blocking
// writes starved the I2C receive path - the board appeared to freeze and came
// back only after a reboot.
class PositionStore {
public:
    // Persist these axes. They must outlive the store - the store keeps
    // references, it does not own the hardware.
    PositionStore(Axis& axisX, Axis& axisY);

    // Read the saved pose and move the axes to it. Call once from setup(),
    // after the axes are attached. When the EEPROM is empty or the record is
    // invalid the axes are left where they are (the neutral pose).
    //
    // On a continuous (wrapping) axis there is no meaningful absolute pose to
    // return to, so restore() is a no-op there and the axis simply boots at
    // neutral. This also guarantees a saved 180 can never park a wrapping axis
    // on an end stop.
    void restore();

    // Call every loop() pass. Tracks movement and saves the pose to EEPROM once
    // the axes have stopped for the debounce interval.
    //
    // Cheap when nothing is happening (two int compares) and cheap while moving
    // (it only refreshes a timestamp). The expensive, blocking EEPROM write
    // happens at most once per still period, and never while the axis moves.
    void update();

    // Force an immediate save of the current pose.
    void saveNow();

    // True when the last save()/restore() found (or wrote) a valid record.
    bool hasSavedData() const;

    // Forget the saved pose (useful when a mechanism is re-homed).
    void clear();

private:
    Axis& axisX_;
    Axis& axisY_;

    bool     haveSaved_    = false;   // a valid record exists in EEPROM
    int      lastSavedX_   = -1;      // pose last written to EEPROM (-1 = none)
    int      lastSavedY_   = -1;
    int      lastSeenX_    = -1;      // pose observed on the previous pass
    int      lastSeenY_    = -1;
    bool     dirty_        = false;   // a change is waiting to be saved
    uint32_t lastChangeMs_ = 0;       // when the pose last changed

    static constexpr int     kAddrMagic   = 0;
    static constexpr int     kAddrVersion = 1;
    static constexpr int     kAddrX       = 2;
    static constexpr int     kAddrY       = 4;
    static constexpr int     kAddrSum     = 6;
    static constexpr uint8_t kMagic       = 0xA5;
    static constexpr uint8_t kVersion     = 1;
    static constexpr uint32_t kSaveDebounceMs = 3000;  // save 3 s after motion stops

    bool     readRecord(int& x, int& y) const;
    void     writeRecord(int x, int y);
    static uint8_t checksum(uint8_t version, uint16_t x, uint16_t y);
};


