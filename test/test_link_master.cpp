#include "Link/LinkMaster.h"
#include "Link/JoystickFrame.h"
#include "Fixture.h"
#include "TestFramework.h"

// LinkMaster is the transmit half: it puts a frame on the bus and reports
// whether the slave acknowledged it.
//
// The acknowledgement is what loop() uses to drive the link LED and to decide
// whether to retry, so it must reflect the bus result honestly - a false "up"
// would leave the operator with a dead rig and a green light.

TEST(LinkMaster_Begin, joins_the_bus_as_master_at_100khz) {
    fixture::reset();
    LinkMaster m(8);

    m.begin();

    CHECK(wire_shim::state().began);
    // Master mode: no own address is claimed.
    CHECK_EQ(wire_shim::state().ownAddress, 0);
    // 100 kHz is the safe standard rate both boards agree on.
    CHECK_EQ(wire_shim::state().clockHz, 100000u);
}

TEST(LinkMaster_Send, addresses_the_configured_slave) {
    fixture::reset();
    LinkMaster m(8);
    m.begin();

    JoystickFrame f;
    f.x = 512;
    f.y = 512;
    f.btn = false;
    m.send(f);

    CHECK_EQ(wire_shim::state().txAddress, 8);
}

TEST(LinkMaster_Send, puts_exactly_one_frame_on_the_wire) {
    fixture::reset();
    LinkMaster m(8);
    m.begin();

    JoystickFrame f;
    f.x = 300;
    f.y = 700;
    f.btn = true;
    m.send(f);

    CHECK_EQ(wire_shim::state().txBuffer.size(), (size_t)JoystickFrame::kSize);

    // And what went out decodes back to what went in.
    JoystickFrame back;
    CHECK(back.decode(wire_shim::state().txBuffer.data()));
    CHECK_EQ(back.x, 300);
    CHECK_EQ(back.y, 700);
    CHECK(back.btn);
}

TEST(LinkMaster_Send, a_successful_send_reports_link_up) {
    fixture::reset();
    LinkMaster m(8);
    m.begin();
    wire_shim::state().endResult = 0;    // ACK

    JoystickFrame f;
    f.x = 512;
    f.y = 512;

    CHECK(m.send(f));
    CHECK(m.isLinkUp());
}

TEST(LinkMaster_Send, a_failed_send_reports_link_down) {
    fixture::reset();
    LinkMaster m(8);
    m.begin();
    wire_shim::state().endResult = 2;    // NAK on address

    JoystickFrame f;
    f.x = 512;
    f.y = 512;

    CHECK(!m.send(f));
    CHECK(!m.isLinkUp());
}

TEST(LinkMaster_Send, link_state_follows_the_latest_result) {
    fixture::reset();
    LinkMaster m(8);
    m.begin();

    JoystickFrame f;
    f.x = 512;
    f.y = 512;

    wire_shim::state().endResult = 0;
    CHECK(m.send(f));
    CHECK(m.isLinkUp());

    wire_shim::state().endResult = 3;
    CHECK(!m.send(f));
    CHECK(!m.isLinkUp());                // recovers from a good state

    wire_shim::state().endResult = 0;
    CHECK(m.send(f));
    CHECK(m.isLinkUp());                 // and back again
}

TEST(LinkMaster_Send, no_frames_sent_before_begin_still_reports_cleanly) {
    fixture::reset();
    LinkMaster m(8);                     // begin() deliberately not called

    JoystickFrame f;
    f.x = 0;
    f.y = 0;
    f.btn = false;

    // The shim has no real bus, so this exercises that send() does not crash
    // and returns a defined value before begin().
    const bool ok = m.send(f);
    CHECK_EQ(ok, m.isLinkUp());
}

TEST(LinkMaster_Send, out_of_range_values_are_not_transmitted_raw) {
    fixture::reset();
    LinkMaster m(8);
    m.begin();

    JoystickFrame f;
    f.x = 5000;                          // beyond the ADC range
    f.y = -100;
    m.send(f);

    // encode() clamps, so a corrupt value never reaches the wire as-is.
    JoystickFrame back;
    CHECK(back.decode(wire_shim::state().txBuffer.data()));
    CHECK_EQ(back.x, 1023);
    CHECK_EQ(back.y, 0);
}

TEST(LinkMaster_Send, every_send_counts_as_one_transaction) {
    fixture::reset();
    LinkMaster m(8);
    m.begin();

    JoystickFrame f;
    f.x = 1;
    f.y = 1;

    m.send(f);
    m.send(f);
    m.send(f);

    CHECK_EQ(wire_shim::state().sendCount, 3);
}
