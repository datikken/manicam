#include "Link/LinkMaster.h"

LinkMaster::LinkMaster(uint8_t slaveAddress, TwoWire& bus)
    : slaveAddr_(slaveAddress), bus_(bus) {}

void LinkMaster::begin() {
    bus_.begin();                    // master: no own address
    bus_.setClock(100000);           // 100 kHz - the safe standard rate
}

bool LinkMaster::send(const JoystickFrame& frame) {
    uint8_t buf[JoystickFrame::kSize];
    const uint8_t n = frame.encode(buf);

    bus_.beginTransmission(slaveAddr_);
    bus_.write(buf, n);
    const uint8_t result = bus_.endTransmission();

    linkUp_ = (result == 0);         // 0 = slave NAKed the address/data
    return linkUp_;
}
