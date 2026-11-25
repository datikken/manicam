#include "Link/LinkSlave.h"
#include "Link/JoystickFrame.h"
#include "Fixture.h"
#include "TestFramework.h"

// LinkSlave is the receive half of the I2C link. Everything awkward about it
// comes from the same place: the I2C driver calls handleReceive() from an
// interrupt, while loop() calls take() - so there are two contexts sharing one
// small buffer.
//
// The tests simulate the ISR by staging bytes in the fake bus and invoking the
// same entry point the driver would, then verify that:
//
//   * a well-formed frame is published and reported as fresh exactly once;
//   * a malformed frame (wrong length, out-of-range value) is rejected and does
//     NOT overwrite the last good pose - that would be the "servos jump
//     somewhere random" failure;
//   * take() reports "fresh" once and then "stale", so loop() can tell a new
//     frame from a repeat;
//   * the diagnostic counters saturate instead of wrapping.

// Friend hook: reaches the private handler that the I2C interrupt would call.
// Must live at global scope - a friend declaration names the global type.
struct LinkSlaveTestHook {
    static void receive(LinkSlave& slave, int count) {
        slave.handleReceive(count);
    }
};

namespace {

// Stage `count` bytes and invoke the ISR entry point exactly as Wire would.
void deliver(LinkSlave& slave, const uint8_t* bytes, int count) {
    auto& bus = wire_shim::state();
    bus.rxBuffer.assign(bytes, bytes + count);
    bus.readPos = 0;
    LinkSlaveTestHook::receive(slave, count);
}

// Deliver a well-formed frame.
void deliverFrame(LinkSlave& slave, int x, int y, bool btn) {
    JoystickFrame f;
    f.x = x;
    f.y = y;
    f.btn = btn;

    uint8_t buf[JoystickFrame::kSize];
    f.encode(buf);
    deliver(slave, buf, JoystickFrame::kSize);
}

}  // namespace

TEST(LinkSlave_Begin, joins_the_bus_under_its_own_address) {
    fixture::reset();
    LinkSlave slave(8);

    slave.begin();

    CHECK(wire_shim::state().began);
    CHECK_EQ(wire_shim::state().ownAddress, 8);
    CHECK_EQ(slave.ownAddress(), 8);
}

TEST(LinkSlave_Receive, no_data_before_the_first_frame) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    CHECK(!slave.hasData());

    JoystickFrame out;
    CHECK(!slave.take(out));
}

TEST(LinkSlave_Receive, a_good_frame_is_published) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    deliverFrame(slave, 300, 700, true);

    CHECK(slave.hasData());

    JoystickFrame out;
    CHECK(slave.take(out));
    CHECK_EQ(out.x, 300);
    CHECK_EQ(out.y, 700);
    CHECK(out.btn);
}

TEST(LinkSlave_Receive, fresh_is_reported_once_then_stale) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();
    deliverFrame(slave, 400, 600, false);

    JoystickFrame out;
    CHECK(slave.take(out));      // first take: this is new
    CHECK(!slave.take(out));     // second take: same data, nothing new

    // The data itself is still returned, so the caller keeps last-known state.
    CHECK_EQ(out.x, 400);
    CHECK_EQ(out.y, 600);
}

TEST(LinkSlave_Receive, a_second_frame_is_fresh_again) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    JoystickFrame out;
    deliverFrame(slave, 400, 600, false);
    CHECK(slave.take(out));

    deliverFrame(slave, 401, 601, false);
    CHECK(slave.take(out));
    CHECK_EQ(out.x, 401);
}

TEST(LinkSlave_Reject, a_short_frame_is_rejected) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    const uint8_t bytes[3] = {0x01, 0x00, 0x01};
    deliver(slave, bytes, 3);

    CHECK(!slave.hasData());
    CHECK_EQ(slave.rejectedCount(), 1);
    CHECK_EQ(slave.acceptedCount(), 0);
}

TEST(LinkSlave_Reject, an_over_long_frame_is_rejected) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    const uint8_t bytes[7] = {0x01, 0x00, 0x01, 0x00, 0x00, 0xAA, 0xBB};
    deliver(slave, bytes, 7);

    CHECK(!slave.hasData());
    CHECK_EQ(slave.rejectedCount(), 1);
}

TEST(LinkSlave_Reject, an_out_of_range_value_is_rejected) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    // xH = 0xFF makes x > 1023: the frame is corrupt.
    const uint8_t bytes[5] = {0xFF, 0xFF, 0x01, 0x00, 0x00};
    deliver(slave, bytes, 5);

    CHECK(!slave.hasData());
    CHECK_EQ(slave.rejectedCount(), 1);
}

TEST(LinkSlave_Reject, a_bad_frame_does_not_clobber_the_last_good_pose) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    deliverFrame(slave, 123, 456, false);   // good
    JoystickFrame out;
    slave.take(out);

    const uint8_t bad[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    deliver(slave, bad, 5);                 // corrupt

    // take() reports "no NEW valid frame", which is correct - a corrupt frame
    // is not news. What matters is that the snapshot still holds the last good
    // pose rather than garbage, so the caller keeps driving the same target.
    CHECK(!slave.take(out));         // nothing fresh
    CHECK_EQ(out.x, 123);            // but still the last known-good value
    CHECK_EQ(out.y, 456);
    CHECK_EQ(slave.acceptedCount(), 1);
    CHECK_EQ(slave.rejectedCount(), 1);
}

TEST(LinkSlave_Reject, the_buffer_is_drained_after_a_bad_frame) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    // A short frame that leaves extra bytes behind must not poison the next
    // good frame: leftovers have to be discarded.
    const uint8_t shortFrame[2] = {0x01, 0x00};
    deliver(slave, shortFrame, 2);

    deliverFrame(slave, 500, 500, false);

    JoystickFrame out;
    CHECK(slave.take(out));
    CHECK_EQ(out.x, 500);
    CHECK_EQ(out.y, 500);
}

TEST(LinkSlave_Stats, accepted_and_rejected_are_counted_separately) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    deliverFrame(slave, 100, 100, false);
    deliverFrame(slave, 200, 200, false);

    const uint8_t bad[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    deliver(slave, bad, 5);

    CHECK_EQ(slave.acceptedCount(), 2);
    CHECK_EQ(slave.rejectedCount(), 1);
}

TEST(LinkSlave_Stats, counters_saturate_instead_of_wrapping) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    // Drive the counter to its ceiling, then past it.
    for (int i = 0; i < 0xFFFF + 10; ++i) {
        const uint8_t bad[1] = {0x00};
        deliver(slave, bad, 1);
    }

    // Saturing at 0xFFFF keeps the diagnostic meaningful; a wrap would make it
    // suddenly claim a handful of rejections mid-session.
    CHECK_EQ(slave.rejectedCount(), 0xFFFF);
}

TEST(LinkSlave_Stats, resetStats_clears_the_counters) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    deliverFrame(slave, 100, 100, false);
    slave.resetStats();

    CHECK_EQ(slave.acceptedCount(), 0);
    CHECK_EQ(slave.rejectedCount(), 0);
}

TEST(LinkSlave_Stats, lastByteCount_records_the_reported_length) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    deliverFrame(slave, 100, 100, false);
    CHECK_EQ(slave.lastByteCount(), (int)JoystickFrame::kSize);
}

TEST(LinkSlave_Receive, zero_length_frame_is_rejected_quietly) {
    fixture::reset();
    LinkSlave slave(8);
    slave.begin();

    auto& bus = wire_shim::state();
    bus.rxBuffer.clear();
    bus.readPos = 0;
    LinkSlaveTestHook::receive(slave, 0);

    CHECK(!slave.hasData());
    CHECK_EQ(slave.rejectedCount(), 1);
}
