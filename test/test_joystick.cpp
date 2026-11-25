#include "Joystick/Joystick.h"
#include "Fixture.h"
#include "TestFramework.h"

// Joystick reads two analog axes and a push button. The tests cover the two
// things that actually misbehave in the field:
//
//   * the dead band, which decides when a resting stick counts as "centred";
//   * button debouncing, which decides when a press is real.
//
// rawX()/rawY() deliberately do NOT apply the dead band: the servo board owns
// that decision, and applying it in both places would mean two dead zones
// fighting each other.

namespace {

constexpr uint8_t kPinX = A0;
constexpr uint8_t kPinY = A1;
constexpr uint8_t kPinBtn = 2;

// A joystick wired to the shim's default pins.
Joystick makeJoystick(uint16_t deadband = 30, uint16_t debounceMs = 30) {
    return Joystick(kPinX, kPinY, kPinBtn, deadband, debounceMs);
}

}  // namespace

TEST(Joystick_Read, raw_values_pass_through_unchanged) {
    fixture::reset();
    Joystick j = makeJoystick();
    j.begin();

    fixture::setAnalog(kPinX, 300);
    fixture::setAnalog(kPinY, 800);

    CHECK_EQ(j.rawX(), 300);
    CHECK_EQ(j.rawY(), 800);
}

TEST(Joystick_Read, raw_values_are_not_snapped_by_the_dead_band) {
    fixture::reset();
    Joystick j = makeJoystick(100);
    j.begin();

    // 530 is only 18 from centre, well inside a 100-count dead band. rawX()
    // must still report the true value so the servo board can compute speed.
    fixture::setAnalog(kPinX, 530);

    CHECK_EQ(j.rawX(), 530);
    CHECK_NE(j.rawX(), Joystick::kCenter);
}

// The dead band is exercised through x()/y(), its public surface. applyDeadband
// is deliberately private: the class is allowed to change how it snaps without
// breaking these tests, as long as the observable behaviour holds.

TEST(Joystick_Deadband, values_inside_the_band_read_as_exactly_centre) {
    fixture::reset();
    Joystick j = makeJoystick(50);
    j.begin();

    fixture::setAnalog(kPinX, Joystick::kCenter);
    CHECK_EQ(j.x(), Joystick::kCenter);

    fixture::setAnalog(kPinX, Joystick::kCenter + 49);
    CHECK_EQ(j.x(), Joystick::kCenter);

    fixture::setAnalog(kPinX, Joystick::kCenter - 49);
    CHECK_EQ(j.x(), Joystick::kCenter);
}

TEST(Joystick_Deadband, values_outside_the_band_pass_through) {
    fixture::reset();
    Joystick j = makeJoystick(50);
    j.begin();

    fixture::setAnalog(kPinX, Joystick::kCenter + 51);
    CHECK_EQ(j.x(), Joystick::kCenter + 51);

    fixture::setAnalog(kPinY, Joystick::kCenter - 51);
    CHECK_EQ(j.y(), Joystick::kCenter - 51);
}

TEST(Joystick_Deadband, the_extremes_are_never_snapped) {
    fixture::reset();
    Joystick j = makeJoystick(200);
    j.begin();

    fixture::setAnalog(kPinX, 0);
    CHECK_EQ(j.x(), 0);

    fixture::setAnalog(kPinX, 1023);
    CHECK_EQ(j.x(), 1023);
}

TEST(Joystick_Deadband, snapping_is_symmetric_around_centre) {
    // The unsigned deadband must not promote to a huge unsigned value when
    // negated: that classic bug leaves the negative side dead everywhere.
    fixture::reset();
    Joystick j = makeJoystick(40);
    j.begin();

    for (int d = 0; d <= 60; d += 5) {
        fixture::setAnalog(kPinX, Joystick::kCenter + d);
        const bool snapHigh = (j.x() == Joystick::kCenter);

        fixture::setAnalog(kPinX, Joystick::kCenter - d);
        const bool snapLow = (j.x() == Joystick::kCenter);

        CHECK_EQ(snapHigh, snapLow);
    }
}

TEST(Joystick_Deadband, setDeadband_takes_effect_immediately) {
    fixture::reset();
    Joystick j = makeJoystick(10);
    j.begin();

    fixture::setAnalog(kPinX, Joystick::kCenter + 30);
    CHECK_NE(j.x(), Joystick::kCenter);       // outside a 10-count band

    j.setDeadband(100);
    CHECK_EQ(j.x(), Joystick::kCenter);       // now inside a 100-count band
    CHECK_EQ(j.deadband(), 100);
}

TEST(Joystick_Button, unpressed_button_reads_false) {
    fixture::reset();
    Joystick j = makeJoystick();
    j.begin();

    fixture::setPin(kPinBtn, HIGH);       // pull-up, not pressed
    fixture::advanceMs(1000);

    CHECK(!j.buttonDown(fixture::nowMs()));
}

TEST(Joystick_Button, a_stable_press_is_accepted_after_the_debounce) {
    fixture::reset();
    Joystick j = makeJoystick(30, 30);
    j.begin();

    fixture::setPin(kPinBtn, HIGH);
    j.buttonDown(0);                      // settle at "released"

    fixture::setPin(kPinBtn, LOW);        // press
    j.buttonDown(10);                     // starts the debounce timer
    j.buttonDown(50);                     // 40 ms later: accepted

    CHECK(j.buttonDown(60));
}

TEST(Joystick_Button, a_press_shorter_than_the_debounce_is_ignored) {
    fixture::reset();
    Joystick j = makeJoystick(30, 30);
    j.begin();

    fixture::setPin(kPinBtn, HIGH);
    j.buttonDown(0);

    fixture::setPin(kPinBtn, LOW);
    j.buttonDown(10);                     // press begins
    fixture::setPin(kPinBtn, HIGH);
    j.buttonDown(20);                     // released again: too quick

    // Bouncing back before the debounce expires must not register a press.
    CHECK(!j.buttonDown(100));
}

TEST(Joystick_Button, bouncing_does_not_register_a_press) {
    fixture::reset();
    Joystick j = makeJoystick(30, 30);
    j.begin();

    fixture::setPin(kPinBtn, HIGH);
    j.buttonDown(0);

    // Rapid alternation faster than the debounce window.
    for (int t = 10; t < 100; t += 5) {
        fixture::setPin(kPinBtn, (t % 10 == 0) ? LOW : HIGH);
        CHECK(!j.buttonDown(t));
    }
}

TEST(Joystick_Button, release_is_debounced_too) {
    fixture::reset();
    Joystick j = makeJoystick(30, 30);
    j.begin();

    fixture::setPin(kPinBtn, HIGH);
    j.buttonDown(0);

    fixture::setPin(kPinBtn, LOW);
    j.buttonDown(10);
    j.buttonDown(50);                     // press accepted

    fixture::setPin(kPinBtn, HIGH);
    j.buttonDown(60);                     // starts the release timer
    j.buttonDown(100);                    // accepted

    CHECK(!j.buttonDown(110));
}

TEST(Joystick_Button, debounce_survives_millis_wraparound) {
    fixture::reset();
    Joystick j = makeJoystick(30, 30);
    j.begin();

    // Just before the 32-bit millis() rollover.
    const uint32_t nearWrap = 0xFFFFFFF0u;

    fixture::setPin(kPinBtn, HIGH);
    j.buttonDown(nearWrap - 100);

    fixture::setPin(kPinBtn, LOW);
    j.buttonDown(nearWrap);               // press begins before the wrap
    j.buttonDown(nearWrap + 40);          // wraps past 0 during the debounce

    // Unsigned subtraction must still measure 40 ms correctly across the wrap.
    CHECK(j.buttonDown(nearWrap + 50));
}

TEST(Joystick_Debug, debugPrint_does_not_crash) {
    fixture::reset();
    Joystick j = makeJoystick();
    j.begin();

    fixture::setAnalog(kPinX, 123);
    fixture::setAnalog(kPinY, 456);

    j.debugPrint();                       // output is stubbed by the shim
    CHECK(true);
}
