#include "Axis/Axis.h"
#include "Fixture.h"
#include "TestFramework.h"

// Axis is the "mechanism" half of the rig: it owns the servo, its travel limits
// and the current angle. The tests below pin down the properties that matter
// for an SG90:
//
//   * it never commands an angle outside its configured limits - the servo has
//     no feedback and no forgiveness, so an out-of-range write is a stall;
//   * it clamps at the ends instead of wrapping, because wrapping makes a
//     positional servo slam its arm across the whole arc;
//   * rotate*() move exactly one degree and report whether they moved;
//   * stepping never sleeps, so the caller keeps control of timing.

TEST(Axis_Construction, defaults_are_centre_within_full_range) {
    fixture::reset();
    Axis a;
    CHECK_EQ(a.getPos(), Axis::kCenterAngle);
    CHECK_EQ(a.minAngle(), Axis::kMinAngleValue);
    CHECK_EQ(a.maxAngle(), Axis::kMaxAngleValue);
}

TEST(Axis_Construction, start_angle_is_clamped_into_the_limits) {
    fixture::reset();
    Axis high(10, 170, 999);
    CHECK_EQ(high.getPos(), 170);

    Axis low(10, 170, -50);
    CHECK_EQ(low.getPos(), 10);
}

TEST(Axis_Construction, swapped_limits_are_tolerated) {
    fixture::reset();
    Axis a(170, 10);                 // spelled backwards on purpose
    CHECK_EQ(a.minAngle(), 10);
    CHECK_EQ(a.maxAngle(), 170);
}

TEST(Axis_Construction, limits_beyond_the_servo_range_are_clamped) {
    fixture::reset();
    Axis a(-40, 500);
    CHECK_EQ(a.minAngle(), 0);
    CHECK_EQ(a.maxAngle(), 180);
}

TEST(Axis_Range, setRange_reclamps_the_current_position) {
    fixture::reset();
    Axis a(0, 180);
    a.setPos(170);

    a.setRange(10, 100);
    CHECK_EQ(a.getPos(), 100);       // pulled inside the new maximum
}

TEST(Axis_Begin, attach_drives_the_start_pose) {
    fixture::reset();
    Axis a(10, 170);
    a.begin(9, 90);

    CHECK(servo_shim::state().attached);
    CHECK_EQ(servo_shim::state().attachPin, 9);
    CHECK_EQ(fixture::servoAngle(), 90);
}

TEST(Axis_SetPos, clamps_at_both_ends) {
    fixture::reset();
    Axis a(10, 170);
    a.begin(9);

    a.setPos(999);
    CHECK_EQ(a.getPos(), 170);
    CHECK_EQ(fixture::servoAngle(), 170);

    a.setPos(-999);
    CHECK_EQ(a.getPos(), 10);
    CHECK_EQ(fixture::servoAngle(), 10);
}

TEST(Axis_SetPos, in_range_value_is_written_verbatim) {
    fixture::reset();
    Axis a(10, 170);
    a.begin(9);

    a.setPos(123);
    CHECK_EQ(a.getPos(), 123);
    CHECK_EQ(fixture::servoAngle(), 123);
}

TEST(Axis_SetPos, never_commands_an_angle_outside_the_limits) {
    fixture::reset();
    Axis a(15, 165);
    a.begin(9);

    a.setPos(1000);
    a.setPos(-1000);

    CHECK(!fixture::servoEverOutside(15, 165));
}

TEST(Axis_Rotate, one_step_moves_exactly_one_degree) {
    fixture::reset();
    Axis a(10, 170);
    a.begin(9, 90);

    CHECK(a.rotateClockwise());
    CHECK_EQ(a.getPos(), 91);

    CHECK(a.rotateCounterClockwise());
    CHECK_EQ(a.getPos(), 90);
}

TEST(Axis_Rotate, stops_at_the_upper_limit_and_reports_it) {
    fixture::reset();
    Axis a(10, 170);
    a.setPos(170);

    CHECK(!a.rotateClockwise());     // already there: no movement
    CHECK_EQ(a.getPos(), 170);
    CHECK(a.blockedClockwise());
    CHECK(a.atEnd());
}

TEST(Axis_Rotate, stops_at_the_lower_limit_and_reports_it) {
    fixture::reset();
    Axis a(10, 170);
    a.setPos(10);

    CHECK(!a.rotateCounterClockwise());
    CHECK_EQ(a.getPos(), 10);
    CHECK(a.blockedCounterClockwise());
    CHECK(a.atEnd());
}

// The regression test for the bug that stalled the servos: driving the axis
// past its limit must clamp, not wrap. A wrap would send 180 -> 0 and make the
// servo swing back across its entire arc.
TEST(Axis_Rotate, never_wraps_after_many_steps_past_the_limit) {
    fixture::reset();
    Axis a(10, 170);
    a.begin(9);                      // attach so every command is observable
    a.setPos(165);

    for (int i = 0; i < 50; ++i) {
        a.rotateClockwise();
    }

    CHECK_EQ(a.getPos(), 170);                  // parked, not wrapped to 10
    CHECK(!fixture::servoEverOutside(10, 170)); // not one illegal command
    CHECK_EQ(fixture::servoAngle(), 170);
}

TEST(Axis_Rotate, never_wraps_when_driven_backwards_past_the_limit) {
    fixture::reset();
    Axis a(10, 170);
    a.setPos(15);

    for (int i = 0; i < 50; ++i) {
        a.rotateCounterClockwise();
    }

    CHECK_EQ(a.getPos(), 10);
    CHECK(!fixture::servoEverOutside(10, 170));
}

TEST(Axis_Rotate, a_full_sweep_visits_every_degree_once) {
    fixture::reset();
    Axis a(10, 170);
    a.setPos(10);

    int steps = 0;
    while (a.rotateClockwise()) {
        ++steps;
    }

    CHECK_EQ(steps, 160);            // 10 -> 170 inclusive, one degree at a time
    CHECK_EQ(a.getPos(), 170);
}

TEST(Axis_Rotate, does_not_advance_the_clock) {
    fixture::reset();
    Axis a(10, 170);
    a.setPos(90);

    const uint32_t before = fixture::nowMs();
    a.rotateClockwise();
    a.rotateCounterClockwise();

    // Pacing belongs to AxisDriver; a sleeping step here would starve loop().
    CHECK_EQ(fixture::nowMs(), before);
}

TEST(Axis_MoveTo, walks_to_the_target_one_degree_at_a_time) {
    fixture::reset();
    Axis a(10, 170);
    a.begin(9, 90);

    a.moveTo(95, 0);                 // zero delay keeps the test instant

    CHECK_EQ(a.getPos(), 95);
    CHECK(!a.isMoving());
}

TEST(Axis_MoveTo, clamps_the_target) {
    fixture::reset();
    Axis a(10, 170);

    a.moveTo(9999, 0);
    CHECK_EQ(a.getPos(), 170);
}

TEST(Axis_MoveTo, honours_the_step_delay_on_the_virtual_clock) {
    fixture::reset();
    Axis a(10, 170);
    a.setPos(90);

    a.moveTo(95, 10);                // 5 degrees * 10 ms

    CHECK_EQ(fixture::nowMs(), 50u);
}

TEST(Axis_Sweep, sweepToMax_ends_on_the_maximum) {
    fixture::reset();
    Axis a(10, 170);
    a.setPos(100);

    a.sweepToMax();
    CHECK_EQ(a.getPos(), 170);
    CHECK(!fixture::servoEverOutside(10, 170));
}

TEST(Axis_Sweep, sweepToMin_ends_on_the_minimum) {
    fixture::reset();
    Axis a(10, 170);
    a.setPos(100);

    a.sweepToMin();
    CHECK_EQ(a.getPos(), 10);
    CHECK(!fixture::servoEverOutside(10, 170));
}

TEST(Axis_State, isMoving_reflects_the_target) {
    fixture::reset();
    Axis a(10, 170);
    a.setPos(90);

    CHECK(!a.isMoving());            // setPos is instantaneous
}

TEST(Axis_State, writing_before_begin_is_safe_and_recorded) {
    fixture::reset();
    Axis a(10, 170);

    a.setPos(120);                   // no servo attached yet

    CHECK_EQ(a.getPos(), 120);       // position tracked even unattached
    CHECK_EQ(servo_shim::state().writeCount, 0);  // nothing sent to hardware
}
