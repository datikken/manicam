#pragma once

// Host-side stand-in for <EEPROM.h>.
//
// Backed by a 1 KiB array, the same size as the Nano's real EEPROM. It also
// counts how many bytes were actually written, which lets a test prove the
// central claim of PositionStore: that a resting rig does not keep writing, and
// that a moving rig does not write at all until it stops.

#include <cstdint>
#include <cstring>
#include <vector>

namespace eeprom_shim {

struct EepromState {
    static constexpr int kSize = 1024;
    uint8_t  data[kSize];
    uint32_t byteWrites = 0;   // total EEPROM.update() calls that changed a byte
    uint32_t readCount  = 0;

    EepromState() { wipe(); }

    void wipe() {
        std::memset(data, 0xFF, sizeof(data));   // erased EEPROM reads 0xFF
        byteWrites = 0;
        readCount  = 0;
    }
};

inline EepromState& state() {
    static EepromState s;
    return s;
}

}  // namespace eeprom_shim

class EEPROMClass {
public:
    uint8_t read(int addr) {
        eeprom_shim::state().readCount++;
        return eeprom_shim::state().data[addr];
    }

    void write(int addr, uint8_t value) {
        if (eeprom_shim::state().data[addr] != value) {
            eeprom_shim::state().byteWrites++;
        }
        eeprom_shim::state().data[addr] = value;
    }

    // Arduino semantics: only write when the value differs, which is what saves
    // cell wear. Counted the same way so tests measure real cell writes.
    void update(int addr, uint8_t value) {
        if (eeprom_shim::state().data[addr] != value) {
            eeprom_shim::state().byteWrites++;
            eeprom_shim::state().data[addr] = value;
        }
    }
};

inline EEPROMClass& eepromInstance() {
    static EEPROMClass e;
    return e;
}

// The application uses the global object `EEPROM`, exactly as <EEPROM.h> provides.
static EEPROMClass& EEPROM = eepromInstance();
