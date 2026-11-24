#include "Joystick/Joystick.h"

Joystick::Joystick(uint8_t pinX, uint8_t pinY, uint8_t pinButton,
                   uint16_t deadband, uint16_t debounceMs)
    : pinX_(pinX),
      pinY_(pinY),
      pinButton_(pinButton),
      deadband_(deadband),
      debounceMs_(debounceMs) {}

void Joystick::begin() {
    // The switch shorts to GND when pressed, so the internal pull-up gives an
    // active-low signal and no external resistor is needed.
    pinMode(pinButton_, INPUT_PULLUP);
    // Analogue pins need no pinMode(); they are inputs by default on AVR.
}

int Joystick::applyDeadband(int value) const {
    const int delta = value - kCenter;
    if (delta > -deadband_ && delta < deadband_) {
        return kCenter;
    }
  return value;
}

bool Joystick::buttonDown(uint32_t now) {
    const bool raw = (digitalRead(pinButton_) == LOW);   // LOW = pressed

    if (raw != btnRaw_) {
        btnRaw_     = raw;
        btnChanged_ = now;
        return btnStable_;              // still bouncing: keep the old state
  }

    if ((uint32_t)(now - btnChanged_) >= debounceMs_ && raw != btnStable_) {
        btnStable_ = raw;               // stable for long enough: accept it
  }
    return btnStable_;
}

void Joystick::debugPrint() const {
    Serial.print("X:");
    Serial.print(rawX());
    Serial.print(" | Y:");
    Serial.print(rawY());
    Serial.print(" | btn:");
    Serial.println(digitalRead(pinButton_) == LOW ? 1 : 0);
}
