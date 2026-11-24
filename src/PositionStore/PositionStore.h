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
// EEPROM cells are rated for ~100k writes. A servo sweeping through the full
// travel would burn through that in weeks, so saving is deliberately lazy:
// a change only starts a debounce timer, and the record is written once the
// axes have been still for kSaveDebounceMs.
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
    // Saving is skipped while either axis wraps, for the same reason: every full
    // turn revisits the same 0..180 values, so writing them would be pointless
    // and would only wear out the EEPROM cells.
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
    bool     dirty_        = false;   // a change is waiting to be saved
    uint32_t lastChangeMs_ = 0;       // when the current dirty spell started

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

