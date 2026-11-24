#include "JoystickFrame.h"

uint8_t JoystickFrame::encode(uint8_t* out) const {
    const uint16_t ux = (uint16_t)constrain(x, kAdcMin, kAdcMax);
    const uint16_t uy = (uint16_t)constrain(y, kAdcMin, kAdcMax);

    out[0] = (uint8_t)((ux >> 8) & 0xFF);   // xH
    out[1] = (uint8_t)(ux & 0xFF);          // xL
    out[2] = (uint8_t)((uy >> 8) & 0xFF);   // yH
    out[3] = (uint8_t)(uy & 0xFF);          // yL
    out[4] = btn ? 1 : 0;
    return kSize;
}

bool JoystickFrame::decode(const uint8_t* in) {
    const uint16_t ux = (uint16_t)(((uint16_t)in[0] << 8) | in[1]);
    const uint16_t uy = (uint16_t)(((uint16_t)in[2] << 8) | in[3]);

    // Values above kAdcMax mean the frame was corrupted or came from a
    // different protocol revision: reject it rather than silently clamping.
    if (ux > (uint16_t)kAdcMax || uy > (uint16_t)kAdcMax) {
        return false;
    }

    x = (int)ux;
    y = (int)uy;
    btn = in[4] != 0;
    return true;
}

void JoystickFrame::clamp() {
    x = constrain(x, kAdcMin, kAdcMax);
    y = constrain(y, kAdcMin, kAdcMax);
}
