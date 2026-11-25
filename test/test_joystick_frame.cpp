#include "Link/JoystickFrame.h"
#include "Fixture.h"
#include "TestFramework.h"

// JoystickFrame is the 5-byte wire format between the two boards. It is the one
// place where corruption turns into "the servos jumped somewhere random", so its
// contract matters more than its size suggests:
//
//   * encode/decode round-trips exactly for every legal value;
//   * byte order is fixed and explicit (high byte first), because both boards
//     must agree and the AVR is little-endian;
//   * out-of-range values are REJECTED on decode, not clamped silently - a
//     corrupt frame should be dropped, not acted on;
//   * the first byte of a frame must never depend on the previous one.

TEST(JoystickFrame_Size, frame_is_five_bytes) {
    CHECK_EQ(JoystickFrame::kSize, 5u);
}

TEST(JoystickFrame_Encode, reports_five_bytes_written) {
    fixture::reset();
    JoystickFrame f;
    f.x = 512;
    f.y = 512;
    f.btn = false;

    uint8_t buf[JoystickFrame::kSize];
    CHECK_EQ(f.encode(buf), 5u);
}

TEST(JoystickFrame_Encode, byte_order_is_high_then_low) {
    fixture::reset();
    JoystickFrame f;
    f.x = 0x0123;                    // 291, comfortably inside 0..1023
    f.y = 0x02AB;                    // 683
    f.btn = true;

    uint8_t buf[JoystickFrame::kSize];
    f.encode(buf);

    CHECK_EQ((int)buf[0], 0x01);     // xH first: both boards must agree
    CHECK_EQ((int)buf[1], 0x23);     // xL
    CHECK_EQ((int)buf[2], 0x02);     // yH
    CHECK_EQ((int)buf[3], 0xAB);     // yL
    CHECK_EQ((int)buf[4], 1);        // button
}

TEST(JoystickFrame_Encode, button_false_encodes_as_zero) {
    fixture::reset();
    JoystickFrame f;
    f.x = 512;
    f.y = 512;
    f.btn = true;
    f.btn = false;

    uint8_t buf[JoystickFrame::kSize];
    f.encode(buf);
    CHECK_EQ((int)buf[4], 0);
}

TEST(JoystickFrame_Encode, out_of_range_values_are_clamped_on_encode) {
    fixture::reset();
    JoystickFrame f;
    f.x = 5000;
    f.y = -300;

    uint8_t buf[JoystickFrame::kSize];
    f.encode(buf);

    JoystickFrame back;
    CHECK(back.decode(buf));
    CHECK_EQ(back.x, 1023);          // clamped to kAdcMax
    CHECK_EQ(back.y, 0);             // clamped to kAdcMin
}

TEST(JoystickFrame_RoundTrip, every_boundary_value_survives) {
    const int values[] = {0, 1, 255, 256, 511, 512, 513, 777, 1022, 1023};

    for (int x : values) {
        for (int y : values) {
            fixture::reset();
            JoystickFrame out;
            out.x = x;
            out.y = y;
            out.btn = true;

            uint8_t buf[JoystickFrame::kSize];
            out.encode(buf);

            JoystickFrame in;
            CHECK(in.decode(buf));
            CHECK_EQ(in.x, x);
            CHECK_EQ(in.y, y);
            CHECK(in.btn);
        }
    }
}

TEST(JoystickFrame_RoundTrip, full_range_sweep_is_lossless) {
    for (int v = 0; v <= 1023; v += 7) {
        fixture::reset();
        JoystickFrame out;
        out.x = v;
        out.y = 1023 - v;
        out.btn = false;

        uint8_t buf[JoystickFrame::kSize];
        out.encode(buf);

        JoystickFrame in;
        CHECK(in.decode(buf));
        CHECK_EQ(in.x, v);
        CHECK_EQ(in.y, 1023 - v);
    }
}

TEST(JoystickFrame_Decode, rejects_x_above_the_adc_maximum) {
    // A corrupted high byte is the classic failure: 0xFF in xH means x > 1023.
    const uint8_t buf[5] = {0xFF, 0xFF, 0x01, 0x00, 0x00};

    JoystickFrame f;
    CHECK(!f.decode(buf));
}

TEST(JoystickFrame_Decode, rejects_y_above_the_adc_maximum) {
    const uint8_t buf[5] = {0x01, 0x00, 0xFF, 0xFF, 0x00};

    JoystickFrame f;
    CHECK(!f.decode(buf));
}

TEST(JoystickFrame_Decode, accepts_exactly_the_maximum) {
    const uint8_t buf[5] = {0x03, 0xFF, 0x03, 0xFF, 0x00};   // 1023, 1023

    JoystickFrame f;
    CHECK(f.decode(buf));
    CHECK_EQ(f.x, 1023);
    CHECK_EQ(f.y, 1023);
}

TEST(JoystickFrame_Decode, accepts_exactly_zero) {
    const uint8_t buf[5] = {0x00, 0x00, 0x00, 0x00, 0x01};

    JoystickFrame f;
    CHECK(f.decode(buf));
    CHECK_EQ(f.x, 0);
    CHECK_EQ(f.y, 0);
    CHECK(f.btn);
}

TEST(JoystickFrame_Decode, any_nonzero_button_byte_counts_as_pressed) {
    const uint8_t buf[5] = {0x01, 0x00, 0x01, 0x00, 0x42};

    JoystickFrame f;
    CHECK(f.decode(buf));
    CHECK(f.btn);
}

TEST(JoystickFrame_Decode, rejected_frame_does_not_modify_the_target) {
    JoystickFrame f;
    f.x = 300;
    f.y = 400;

    const uint8_t bad[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    CHECK(!f.decode(bad));

    // A rejected frame must leave the previous value intact, so the caller can
    // keep last-known-good state.
    CHECK_EQ(f.x, 300);
    CHECK_EQ(f.y, 400);
}

TEST(JoystickFrame_Clamp, pulls_both_axes_into_range) {
    fixture::reset();
    JoystickFrame f;
    f.x = 9999;
    f.y = -5;

    f.clamp();

    CHECK_EQ(f.x, 1023);
    CHECK_EQ(f.y, 0);
}

TEST(JoystickFrame_Clamp, leaves_in_range_values_untouched) {
    fixture::reset();
    JoystickFrame f;
    f.x = 512;
    f.y = 0;

    f.clamp();

    CHECK_EQ(f.x, 512);
    CHECK_EQ(f.y, 0);
}

TEST(JoystickFrame_Decode, is_stateless_across_frames) {
    // Decoding a good frame after a bad one must work: no lingering state.
    JoystickFrame f;

    const uint8_t bad[5]  = {0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    const uint8_t good[5] = {0x02, 0x00, 0x01, 0x00, 0x01};

    CHECK(!f.decode(bad));
    CHECK(f.decode(good));
    CHECK_EQ(f.x, 512);
    CHECK_EQ(f.y, 256);
    CHECK(f.btn);
}
