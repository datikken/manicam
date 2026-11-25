#include "PositionStore/PositionStore.h"
#include "Axis/Axis.h"
#include "Fixture.h"
#include "TestFramework.h"

// PositionStore saves the pose to EEPROM so the rig comes back to where it was.
//
// Two properties matter far more than the encoding:
//
//   1. A VALID pose survives a reboot, and a corrupt or half-written record is
//      rejected rather than acted on - restoring garbage would drive both axes
//      to a random angle.
//   2. Saving must never happen while an axis is moving, and never more than
//      once per still period. Writing in a loop blocks the CPU for ~3.3 ms per
//      changed byte and starves the I2C receive path: that was the "freezes
//      under a fast stress test" failure.

namespace {

struct Store {
    Axis          axisX;
    Axis          axisY;
    PositionStore store;

    Store() : axisX(10, 170), axisY(10, 170), store(axisX, axisY) {
        axisX.begin(9, 90);
        axisY.begin(10, 90);
    }
};

// Run update() the way loop() would, for a given span of virtual time.
void pump(Store& s, uint32_t durationMs) {
    for (uint32_t t = 0; t < durationMs; ++t) {
        s.store.update();
        fixture::advanceMs(1);
    }
}

}  // namespace

TEST(PositionStore_Restore, empty_eeprom_leaves_the_neutral_pose) {
    fixture::reset();
    Store s;

    s.store.restore();

    CHECK(!s.store.hasSavedData());
    CHECK_EQ(s.axisX.getPos(), 90);      // untouched
    CHECK_EQ(s.axisY.getPos(), 90);
}

TEST(PositionStore_RoundTrip, a_saved_pose_is_restored_exactly) {
    fixture::reset();
    int savedX = 0;
    int savedY = 0;
    {
        Store s;
        s.axisX.setPos(40);
        s.axisY.setPos(155);
        s.store.saveNow();

        savedX = s.axisX.getPos();
        savedY = s.axisY.getPos();
        CHECK(s.store.hasSavedData());
    }

    // "Reboot": brand-new objects, same EEPROM.
    Store fresh;
    fresh.store.restore();

    CHECK(fresh.store.hasSavedData());
    CHECK_EQ(fresh.axisX.getPos(), savedX);
    CHECK_EQ(fresh.axisY.getPos(), savedY);
}

TEST(PositionStore_RoundTrip, boundary_angles_survive) {
    for (int angle : {10, 11, 90, 169, 170}) {
        fixture::reset();
        {
            Store s;
            s.axisX.setPos(angle);
            s.axisY.setPos(angle);
            s.store.saveNow();
        }
        Store fresh;
        fresh.store.restore();
        CHECK_EQ(fresh.axisX.getPos(), angle);
        CHECK_EQ(fresh.axisY.getPos(), angle);
    }
}

TEST(PositionStore_Validation, a_wiped_magic_byte_means_no_data) {
    fixture::reset();
    Store s;
    s.axisX.setPos(50);
    s.store.saveNow();

    eeprom_shim::state().data[0] = 0xFF;   // erase the magic marker

    Store fresh;
    fresh.store.restore();
    CHECK(!fresh.store.hasSavedData());
    CHECK_EQ(fresh.axisX.getPos(), 90);
}

TEST(PositionStore_Validation, a_corrupted_payload_is_rejected) {
    fixture::reset();
    Store s;
    s.axisX.setPos(50);
    s.axisY.setPos(60);
    s.store.saveNow();

    // Flip a bit in the X payload: the checksum must catch it.
    eeprom_shim::state().data[2] ^= 0x01;

    Store fresh;
    fresh.store.restore();

    CHECK(!fresh.store.hasSavedData());
    CHECK_EQ(fresh.axisX.getPos(), 90);    // left at neutral, not corrupted
}

TEST(PositionStore_Validation, an_out_of_range_angle_is_rejected) {
    fixture::reset();

    // Forge a self-consistent record whose angle is beyond the servo range.
    auto& e = eeprom_shim::state();
    const uint8_t version = 1;
    const uint16_t badX = 9999;
    const uint16_t badY = 90;

    e.data[0] = 0xA5;
    e.data[1] = version;
    e.data[2] = (uint8_t)(badX & 0xFF);
    e.data[3] = (uint8_t)(badX >> 8);
    e.data[4] = (uint8_t)(badY & 0xFF);
    e.data[5] = (uint8_t)(badY >> 8);
    e.data[6] = (uint8_t)(version ^ (uint8_t)(badX & 0xFF) ^ (uint8_t)(badX >> 8) ^
                          (uint8_t)(badY & 0xFF) ^ (uint8_t)(badY >> 8));

    Store s;
    s.store.restore();

    CHECK(!s.store.hasSavedData());
}

TEST(PositionStore_Validation, an_old_version_is_rejected) {
    fixture::reset();
    Store s;
    s.axisX.setPos(50);
    s.store.saveNow();

    eeprom_shim::state().data[1] = 99;     // some future/other format

    Store fresh;
    fresh.store.restore();
    CHECK(!fresh.store.hasSavedData());
}

TEST(PositionStore_Clear, clear_invalidates_the_record) {
    fixture::reset();
    Store s;
    s.axisX.setPos(50);
    s.store.saveNow();
    CHECK(s.store.hasSavedData());

    s.store.clear();
    CHECK(!s.store.hasSavedData());

    Store fresh;
    fresh.store.restore();
    CHECK_EQ(fresh.axisX.getPos(), 90);
}

// --- the stress-test regression ------------------------------------------

TEST(PositionStore_Debounce, does_not_save_while_the_pose_keeps_changing) {
    fixture::reset();
    Store s;
    const uint32_t before = eeprom_shim::state().byteWrites;

    // An axis that moves one degree per millisecond for 10 seconds: far longer
    // than the 3 s debounce, so a naive timer would have fired repeatedly.
    for (int i = 0; i < 10000; ++i) {
        if (s.axisX.getPos() < 170) {
            s.axisX.rotateClockwise();
        } else {
            s.axisX.rotateCounterClockwise();
        }
        s.store.update();
        fixture::advanceMs(1);
    }

    // Not a single byte may have been written while it was moving.
    CHECK_EQ(eeprom_shim::state().byteWrites, before);
}

TEST(PositionStore_Debounce, saves_exactly_once_after_the_axes_settle) {
    fixture::reset();
    Store s;

    s.axisX.setPos(120);
    pump(s, 3500);                         // past the 3 s debounce

    const uint32_t afterFirst = eeprom_shim::state().byteWrites;
    CHECK_GE((int)afterFirst, 1);

    // Keep sitting still for another 10 s: no further writes at all.
    pump(s, 10000);
    CHECK_EQ(eeprom_shim::state().byteWrites, afterFirst);
}

TEST(PositionStore_Debounce, a_stress_run_does_not_storm_the_eeprom) {
    fixture::reset();
    Store s;

    // 200 stop/start cycles of one second each. Before the debounce fix this
    // wrote EEPROM on a tight loop; now only genuinely-settled poses are saved.
    for (int cycle = 0; cycle < 200; ++cycle) {
        s.axisX.setPos(20 + (cycle % 140));
        pump(s, 1000);                     // never still for 3 s
    }

    // Nothing settled for long enough, so nothing should have been written.
    CHECK_EQ(eeprom_shim::state().byteWrites, 0u);
}

TEST(PositionStore_Debounce, a_settled_pose_is_written_only_when_it_changes) {
    fixture::reset();
    Store s;

    s.axisX.setPos(100);
    pump(s, 4000);                         // settles, writes once
    const uint32_t first = eeprom_shim::state().byteWrites;

    pump(s, 20000);                        // still for 20 s more
    CHECK_EQ(eeprom_shim::state().byteWrites, first);

    s.axisX.setPos(101);                   // a genuinely new pose
    pump(s, 4000);
    CHECK_GE((int)eeprom_shim::state().byteWrites, (int)first + 1);
}

TEST(PositionStore_Debounce, a_short_move_that_returns_home_writes_nothing) {
    fixture::reset();
    Store s;

    s.store.saveNow();                     // establish a baseline pose
    const uint32_t baseline = eeprom_shim::state().byteWrites;

    // Nudge out and back within the debounce window.
    s.axisX.setPos(120);
    pump(s, 100);
    s.axisX.setPos(90);
    pump(s, 100);

    pump(s, 5000);                         // settle: same pose as before

    CHECK_EQ(eeprom_shim::state().byteWrites, baseline);
}

TEST(PositionStore_Restore, restore_does_not_immediately_trigger_a_save) {
    fixture::reset();
    {
        Store s;
        s.axisX.setPos(130);
        s.axisY.setPos(45);
        s.store.saveNow();
    }

    Store fresh;
    fresh.store.restore();
    const uint32_t afterRestore = eeprom_shim::state().byteWrites;

    pump(fresh, 10000);                    // sit still after a restore

    CHECK_EQ(eeprom_shim::state().byteWrites, afterRestore);
}

TEST(PositionStore_Restore, restored_pose_is_not_re_saved) {
    fixture::reset();
    {
        Store s;
        s.axisX.setPos(33);
        s.store.saveNow();
    }

    Store fresh;
    fresh.store.restore();
    const uint32_t afterRestore = eeprom_shim::state().byteWrites;

    pump(fresh, 4000);

    // restore() seeds the "last seen" pose, so no redundant write follows.
    CHECK_EQ(eeprom_shim::state().byteWrites, afterRestore);
}
