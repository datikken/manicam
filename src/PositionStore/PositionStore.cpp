#include "PositionStore/PositionStore.h"
#include <EEPROM.h>

PositionStore::PositionStore(Axis& axisX, Axis& axisY)
    : axisX_(axisX), axisY_(axisY) {}

uint8_t PositionStore::checksum(uint8_t version, uint16_t x, uint16_t y) {
    return (uint8_t)(version ^ (uint8_t)(x & 0xFF) ^ (uint8_t)(x >> 8) ^
                     (uint8_t)(y & 0xFF) ^ (uint8_t)(y >> 8));
}

bool PositionStore::hasSavedData() const {
    return haveSaved_;
}

bool PositionStore::readRecord(int& x, int& y) const {
    if (EEPROM.read(kAddrMagic) != kMagic) {
        return false;                       // fresh board, never written
    }
    if (EEPROM.read(kAddrVersion) != kVersion) {
        return false;                       // written by an older firmware
    }

    const uint16_t rawX = (uint16_t)EEPROM.read(kAddrX) |
                          (uint16_t)((uint16_t)EEPROM.read(kAddrX + 1) << 8);
    const uint16_t rawY = (uint16_t)EEPROM.read(kAddrY) |
                          (uint16_t)((uint16_t)EEPROM.read(kAddrY + 1) << 8);

    if (EEPROM.read(kAddrSum) != checksum(kVersion, rawX, rawY)) {
        return false;                       // interrupted or corrupted write
    }
    if (rawX > (uint16_t)Axis::kMaxAngleValue ||
        rawY > (uint16_t)Axis::kMaxAngleValue) {
        return false;                       // out of range: not our data
    }

    x = (int)rawX;
    y = (int)rawY;
    return true;
}

void PositionStore::writeRecord(int x, int y) {
    const uint16_t rawX = (uint16_t)x;
    const uint16_t rawY = (uint16_t)y;

    // EEPROM.update() writes the byte only when it differs, which saves wear.
    EEPROM.update(kAddrVersion, kVersion);
    EEPROM.update(kAddrX, (uint8_t)(rawX & 0xFF));
    EEPROM.update(kAddrX + 1, (uint8_t)(rawX >> 8));
    EEPROM.update(kAddrY, (uint8_t)(rawY & 0xFF));
    EEPROM.update(kAddrY + 1, (uint8_t)(rawY >> 8));
    EEPROM.update(kAddrSum, checksum(kVersion, rawX, rawY));

    // The magic marker is written LAST: an interrupted save therefore leaves
    // the record invalid instead of half-applied.
    EEPROM.update(kAddrMagic, kMagic);

    lastSavedX_ = x;
    lastSavedY_ = y;
    haveSaved_  = true;
    dirty_      = false;
}

void PositionStore::restore() {
    // A continuous axis has no absolute pose: whatever value was stored refers
    // to a position it has long since wrapped past. Leave it at neutral.
    if (axisX_.wraps() || axisY_.wraps()) {
        lastSavedX_ = -1;
        lastSavedY_ = -1;
        haveSaved_  = false;
        dirty_      = false;
        return;
    }

    int x = 0;
    int y = 0;

    if (!readRecord(x, y)) {
        // Nothing usable stored: keep the neutral pose the axes booted with and
        // remember that there is nothing saved yet.
        lastSavedX_ = -1;
        lastSavedY_ = -1;
        haveSaved_  = false;
        dirty_      = false;
        return;
    }

    axisX_.setPos(x);
    axisY_.setPos(y);

    lastSavedX_ = x;
    lastSavedY_ = y;
    haveSaved_  = true;
    dirty_      = false;
}

void PositionStore::update() {
    // Nothing to persist for an axis that wraps: the angle repeats every turn.
    if (axisX_.wraps() || axisY_.wraps()) {
        return;
    }

    const int x = axisX_.getPos();
    const int y = axisY_.getPos();

    if (x == lastSavedX_ && y == lastSavedY_) {
        dirty_ = false;               // back where we started: nothing to save
        return;
    }

    if (!dirty_) {
        dirty_        = true;         // first change of this spell
        lastChangeMs_ = millis();
        return;
    }

    if ((uint32_t)(millis() - lastChangeMs_) >= kSaveDebounceMs) {
        saveNow();                    // been still long enough: persist it
    }
}

void PositionStore::saveNow() {
    writeRecord(axisX_.getPos(), axisY_.getPos());
}

void PositionStore::clear() {
    EEPROM.update(kAddrMagic, 0x00);   // invalidation is a single byte write
    haveSaved_  = false;
    lastSavedX_ = -1;
    lastSavedY_ = -1;
    dirty_      = false;
}

