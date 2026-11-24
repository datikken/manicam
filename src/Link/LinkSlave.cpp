#include "Link/LinkSlave.h"

namespace {
// The ISR trampoline is static (the AVR I2C driver passes no user pointer), so
// it has to find the one live instance through this file-scope pointer.
LinkSlave* g_activeSlave = nullptr;
}  // namespace

LinkSlave::LinkSlave(uint8_t ownAddress, TwoWire& bus)
    : ownAddr_(ownAddress), bus_(bus) {}

void LinkSlave::begin() {
    g_activeSlave = this;
    bus_.begin(ownAddr_);
    bus_.onReceive(&LinkSlave::onReceiveTrampoline);
}

void LinkSlave::onReceiveTrampoline(int count) {
    if (g_activeSlave != nullptr) {
        g_activeSlave->handleReceive(count);
    }
}

void LinkSlave::handleReceive(int count) {
    JoystickFrame frame;
    lastCount_ = count;
    bool ok = (count == (int)JoystickFrame::kSize);

    if (ok) {
        uint8_t buf[JoystickFrame::kSize];
        for (uint8_t i = 0; i < JoystickFrame::kSize; ++i) {
            buf[i] = (uint8_t)bus_.read();
        }
        ok = frame.decode(buf);
    }

    // Whatever happened, drain the buffer so the next frame starts clean.
    while (bus_.available() > 0) {
        (void)bus_.read();
    }

    if (ok) {
        ++accepted_;
        publish(frame);
    } else {
        ++rejected_;
    }
}

void LinkSlave::publish(const JoystickFrame& frame) {
    const uint8_t oldSREG = SREG;
    cli();                                  // atomic publish of (x, y, btn)
    pendingX_   = frame.x;
    pendingY_   = frame.y;
    pendingBtn_ = frame.btn;
    pendingNew_ = true;
    hasData_    = true;
    SREG = oldSREG;
}

bool LinkSlave::take(JoystickFrame& out) {
    const uint8_t oldSREG = SREG;
    cli();                                  // atomic snapshot
    const int  x   = pendingX_;
    const int  y   = pendingY_;
    const bool btn = pendingBtn_;
    const bool fresh = pendingNew_;
    pendingNew_ = false;
    SREG = oldSREG;

    out.x   = x;
    out.y   = y;
    out.btn = btn;
    return fresh && hasData_;
}

