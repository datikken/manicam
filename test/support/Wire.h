#pragma once

// Host-side stand-in for <Wire.h>.
//
// The real Wire drives an I2C bus. Here the "bus" is a queue of bytes that a
// test fills directly, which lets LinkSlave be exercised the way the ISR would
// exercise it: a byte count, then that many bytes. LinkMaster's send() is
// captured so a test can decode what the master put on the wire.
//
// endTransmission()'s return value is settable, so both the "slave ACKed" and
// "slave absent / NAKed" paths can be tested.

#include <cstdint>
#include <vector>

namespace wire_shim {

struct WireState {
    // Receive side (slave): bytes the test has staged, plus a read cursor.
    std::vector<uint8_t> rxBuffer;
    size_t               readPos = 0;

    // Transmit side (master): bytes written by beginTransmission/write.
    std::vector<uint8_t> txBuffer;
    uint8_t              txAddress = 0;

    // Result returned by endTransmission(): 0 = success/ACK.
    uint8_t endResult = 0;

    // Recorded configuration.
    bool    began       = false;
    uint8_t ownAddress  = 0;
    uint32_t clockHz    = 0;
    int     sendCount   = 0;
    int     beginCount  = 0;

    void reset() {
        rxBuffer.clear();
        txBuffer.clear();
        readPos    = 0;
        txAddress  = 0;
        endResult  = 0;
        began      = false;
        ownAddress = 0;
        clockHz    = 0;
        sendCount  = 0;
        beginCount = 0;
    }
};

inline WireState& state() {
    static WireState s;
    return s;
}

}  // namespace wire_shim

class TwoWire {
public:
    // Slave mode: join the bus under an address.
    void begin(uint8_t address) {
        wire_shim::state().began       = true;
        wire_shim::state().ownAddress  = address;
        wire_shim::state().beginCount++;
    }

    // Master mode: no own address.
    void begin() {
        wire_shim::state().began = true;
        wire_shim::state().beginCount++;
    }

    void setClock(uint32_t hz) { wire_shim::state().clockHz = hz; }

    void onReceive(void (*)(int)) {}

    // --- receive side -----------------------------------------------------
    int available() const {
        const auto& s = wire_shim::state();
        return (int)(s.rxBuffer.size() - s.readPos);
    }

    int read() {
        auto& s = wire_shim::state();
        if (s.readPos >= s.rxBuffer.size()) {
            return -1;
        }
        return s.rxBuffer[s.readPos++];
    }

    // --- transmit side ----------------------------------------------------
    void beginTransmission(uint8_t address) {
        wire_shim::state().txAddress = address;
        wire_shim::state().txBuffer.clear();
    }

    size_t write(uint8_t value) {
        wire_shim::state().txBuffer.push_back(value);
        return 1;
    }

    size_t write(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            wire_shim::state().txBuffer.push_back(data[i]);
        }
        return len;
    }

    uint8_t endTransmission() {
        wire_shim::state().sendCount++;
        return wire_shim::state().endResult;
    }
};

inline TwoWire& wireInstance() {
    static TwoWire w;
    return w;
}

// The application declares its defaults as `TwoWire& bus = Wire`, so a global
// object named Wire is required.
static TwoWire& Wire = wireInstance();
