#include "AxisDriver/AxisDriver.h"
#include "Axis/Axis.h"
#include "Fixture.h"
#include "TestFramework.h"

namespace {

// Build an attached axis plus a driver, ready to be fed stick readings.
struct Rig {
    Axis        axis;
    AxisDriver  driver;

    explicit Rig(int lo = 10, int hi = 170)
        : axis(lo, hi), driver(axis) {
        axis.begin(9, 90);
    }
};

// Feed a reading once per millisecond, simulating a loop() that runs far faster
// than the pacing interval. Returns how many degrees the axis actually moved.
int driveFor(Rig& rig, int stick, uint32_t durationMs) {
    const int start = rig.axis.getPos();
    for (uint32_t t = 0; t < durationMs; ++t) {
        rig.driver.update(stick);
        fixture::advanceMs(1);
    }
    return rig.axis.getPos() - start;
}

}  // namespace

// AxisDriver turns "how far the stick is pushed" into "how fast to step". The
// tests pin down the properties an operator actually feels:
//
//   * centred stick means no motion, with hysteresis around the dead zone;
//   * both directions are symmetric;
//   * pushing harder is never slower;
//   * the axis stops at its limit instead of grinding past it;
//   * one update() never moves more than a single degree, so loop() stays
//     responsive and I2C keeps being serviced.

TEST(AxisDriver_DeadZone, centred_stick_does_not_move) {
    fixture::reset();
    Rig rig;

    driveFor(rig, AxisDriver::kDefaultCenter, 500);

    CHECK_EQ(rig.axis.getPos(), 90);
}

TEST(AxisDriver_DeadZone, small_jitter_inside_the_band_is_ignored) {
    fixture::reset();
    Rig rig;

    // +-24 counts is inside the default dead zone of 25.
    for (int v = 488; v <= 536; v += 4) {
        rig.driver.update(v);
        fixture::advanceMs(100);
    }

    CHECK_EQ(rig.axis.getPos(), 90);
}

TEST(AxisDriver_DeadZone, just_outside_the_band_moves) {
    fixture::reset();
    Rig rig;

    // Give it longer than the slowest step period (80 ms): the first step can
    // only happen once a full interval has elapsed, because lastStepMs_ starts
    // at 0 and the clock starts at 0.
    const int moved = driveFor(rig, AxisDriver::kDefaultCenter + 40, 200);

    CHECK(moved > 0);
}

TEST(AxisDriver_Direction, above_centre_increases_the_angle) {
    fixture::reset();
    Rig rig;
    const int before = rig.axis.getPos();

    driveFor(rig, 700, 300);

    CHECK(rig.axis.getPos() > before);
}

TEST(AxisDriver_Direction, below_centre_decreases_the_angle) {
    fixture::reset();
    Rig rig;
    const int before = rig.axis.getPos();

    driveFor(rig, 324, 300);

    CHECK(rig.axis.getPos() < before);
}

TEST(AxisDriver_Direction, both_directions_are_symmetric) {
    fixture::reset();
    {
        Rig up;
        const int before = up.axis.getPos();
        driveFor(up, AxisDriver::kDefaultCenter + 250, 400);
        const int upDelta = up.axis.getPos() - before;

        fixture::reset();

        Rig down;
        const int before2 = down.axis.getPos();
        driveFor(down, AxisDriver::kDefaultCenter - 250, 400);
        const int downDelta = down.axis.getPos() - before2;

        CHECK_EQ(upDelta, -downDelta);   // same speed, opposite sign
    }
}

TEST(AxisDriver_Speed, harder_push_is_faster) {
    fixture::reset();
    Rig gentle;
    const int g = driveFor(gentle, AxisDriver::kDefaultCenter + 150, 800);

    fixture::reset();
    Rig hard;
    const int h = driveFor(hard, AxisDriver::kDefaultCenter + 450, 800);

    CHECK(h > g);
}

TEST(AxisDriver_Speed, speed_is_monotonic_across_the_whole_stick) {
    // Delay must never increase as deflection grows: pushing further can only
    // make the axis go faster.
    fixture::reset();
    Rig rig;

    uint16_t previous = 0xFFFF;
    for (int delta = 30; delta <= 500; delta += 10) {
        const uint16_t d = rig.driver.speedDelayFor(AxisDriver::kDefaultCenter + delta);
        CHECK_LE(d, previous);
        previous = d;
    }
}

TEST(AxisDriver_Speed, full_deflection_reaches_the_fastest_period) {
    fixture::reset();
    Rig rig;

    const uint16_t delay = rig.driver.speedDelayFor(
        AxisDriver::kDefaultCenter + AxisDriver::kDefaultFullDeflection);

    // The documented top speed is ~143 deg/s, i.e. 7 ms per degree.
    CHECK_EQ(delay, 7);
    CHECK_NEAR(rig.driver.speedFor(AxisDriver::kDefaultCenter + 400), 142.0, 3.0);
}

TEST(AxisDriver_Speed, beyond_full_deflection_stays_capped) {
    fixture::reset();
    Rig rig;

    const uint16_t a = rig.driver.speedDelayFor(AxisDriver::kDefaultCenter + 400);
    const uint16_t b = rig.driver.speedDelayFor(AxisDriver::kDefaultCenter + 511);

    CHECK_EQ(a, b);
}

// The requirement that started this: a small nudge must stay slow even though
// the top end is fast.
TEST(AxisDriver_Speed, a_gentle_nudge_stays_slow) {
    fixture::reset();
    Rig rig;

    const uint16_t nudge = rig.driver.speedDelayFor(AxisDriver::kDefaultCenter + 30);
    CHECK_GE(nudge, 75);             // at least 75 ms per degree: a crawl
}

TEST(AxisDriver_Speed, inside_the_dead_zone_reports_zero_speed) {
    fixture::reset();
    Rig rig;

    CHECK_EQ(rig.driver.speedFor(AxisDriver::kDefaultCenter), 0);
}

// Documented quirk: lastStepMs_ starts at 0 and so does the virtual clock, so a
// brand-new driver refuses its first step until one full period has elapsed. On
// real hardware millis() is already well past 0 by the time loop() runs, so this
// only ever costs one frame. Pinned here so it is a known behaviour.
TEST(AxisDriver_Pacing, a_fresh_driver_waits_one_full_period_before_stepping) {
    fixture::reset();
    Rig rig;

    const int before = rig.axis.getPos();

    CHECK(!rig.driver.update(1023));          // t = 0: too early
    fixture::advanceMs(7);                    // the fastest period
    CHECK(rig.driver.update(1023));           // now allowed

    CHECK_EQ(rig.axis.getPos() - before, 1);
}

TEST(AxisDriver_Pacing, a_single_update_moves_at_most_one_degree) {
    fixture::reset();
    Rig rig;
    fixture::advanceMs(1000);                 // well past the first interval

    const int before = rig.axis.getPos();
    rig.driver.update(1023);
    CHECK_EQ(rig.axis.getPos() - before, 1);

    // Even after a long gap, one call is still only one degree.
    fixture::advanceMs(1000);
    rig.driver.update(1023);
    CHECK_EQ(rig.axis.getPos() - before, 2);
}

TEST(AxisDriver_Pacing, repeated_updates_within_the_interval_do_not_move) {
    fixture::reset();
    Rig rig;

    rig.driver.update(1023);         // one degree, lastStepMs = 0
    const int after = rig.axis.getPos();

    // Six more calls inside the same millisecond must all be refused.
    for (int i = 0; i < 6; ++i) {
        CHECK(!rig.driver.update(1023));
    }

    CHECK_EQ(rig.axis.getPos(), after);
}

TEST(AxisDriver_Pacing, update_never_advances_the_clock) {
    fixture::reset();
    Rig rig;

    const uint32_t before = fixture::nowMs();
    rig.driver.update(1023);

    // Pacing must not sleep: loop() has to keep servicing I2C.
    CHECK_EQ(fixture::nowMs(), before);
}

TEST(AxisDriver_Limits, driving_hard_stops_at_the_maximum) {
    fixture::reset();
    Rig rig;

    driveFor(rig, 1023, 5000);

    CHECK_EQ(rig.axis.getPos(), 170);
    CHECK(!fixture::servoEverOutside(10, 170));
}

TEST(AxisDriver_Limits, driving_hard_stops_at_the_minimum) {
    fixture::reset();
    Rig rig;

    driveFor(rig, 0, 5000);

    CHECK_EQ(rig.axis.getPos(), 10);
    CHECK(!fixture::servoEverOutside(10, 170));
}

TEST(AxisDriver_Limits, pushing_against_the_stop_reports_no_movement) {
    fixture::reset();
    Rig rig;
    driveFor(rig, 1023, 5000);       // parked at 170

    fixture::advanceMs(100);

    CHECK(!rig.driver.update(1023));
}

TEST(AxisDriver_Calibration, calibrate_moves_the_resting_point) {
    fixture::reset();
    Rig rig;

    rig.driver.calibrate(700);
    CHECK_EQ(rig.driver.getCenter(), 700);

    driveFor(rig, 700, 500);         // now 700 is "centred"
    CHECK_EQ(rig.axis.getPos(), 90);
}

TEST(AxisDriver_Calibration, implausible_reading_is_rejected) {
    fixture::reset();
    Rig rig;

    // A stick held during boot would otherwise become "the middle".
    rig.driver.calibrate(50);        // below kPlausibleMin

    CHECK_EQ(rig.driver.getCenter(), 50);  // calibrate() is unconditional

    // calibrateFrom() is the guarded one:
    Rig rig2;
    rig2.driver.calibrateFrom([] { return 50; }, 4);
    CHECK_EQ(rig2.driver.getCenter(), AxisDriver::kDefaultCenter);
}

TEST(AxisDriver_Calibration, calibrateFrom_averages_plausible_readings) {
    fixture::reset();
    Rig rig;

    rig.driver.calibrateFrom([] { return 520; }, 8);

    CHECK_EQ(rig.driver.getCenter(), 520);
}

TEST(AxisDriver_Settings, setFullDeflection_reaches_max_sooner) {
    fixture::reset();
    Rig rig;
    rig.driver.setFullDeflection(200);

    CHECK_EQ(rig.driver.speedDelayFor(AxisDriver::kDefaultCenter + 200), 7);
}
