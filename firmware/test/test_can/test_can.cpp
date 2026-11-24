#include <unity.h>

#include "hal/mock/mock_can.hpp"

using namespace hal;

namespace {

uint32_t makeDroneCanMessageId(uint8_t priority, uint16_t type_id, uint8_t node_id) {
    return (static_cast<uint32_t>(priority) << 24) |
           (static_cast<uint32_t>(type_id) << 8) |
           static_cast<uint32_t>(node_id);
}

CanFrame makeFrame(uint32_t id, uint8_t length) {
    CanFrame frame{};
    frame.id = id;
    frame.length = length;
    frame.received_at_us = 123456;
    for (uint8_t index = 0; index < length && index < CAN_MAX_DATA_LENGTH; ++index) {
        frame.data[index] = static_cast<uint8_t>(index + 1);
    }
    return frame;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_receive_is_non_blocking_when_queue_is_empty() {
    MockCan can;
    CanFrame frame{};

    TEST_ASSERT_FALSE(can.receive(frame));
}

void test_received_frame_is_preserved() {
    MockCan can;
    const CanFrame expected = makeFrame(0x12345678, 8);
    CanFrame actual{};

    TEST_ASSERT_TRUE(can.injectReceivedFrame(expected));
    TEST_ASSERT_TRUE(can.receive(actual));
    TEST_ASSERT_EQUAL_MEMORY(&expected, &actual, sizeof(CanFrame));
    TEST_ASSERT_FALSE(can.receive(actual));
}

void test_filter_matches_dronecan_type_id_with_mask() {
    MockCan can;
    const CanFilter filter{
        static_cast<uint32_t>(PM08_BATTERY_INFO_TYPE_ID) << 8,
        DRONECAN_MESSAGE_TYPE_MASK,
    };
    const CanFrame wrong_type = makeFrame(makeDroneCanMessageId(16, PM08_NODE_STATUS_TYPE_ID, 42), 1);
    const CanFrame service_frame = makeFrame(
        makeDroneCanMessageId(16, PM08_BATTERY_INFO_TYPE_ID, 42) | 0x80U, 1);
    const CanFrame matching_a = makeFrame(makeDroneCanMessageId(5, PM08_BATTERY_INFO_TYPE_ID, 12), 1);
    const CanFrame matching_b = makeFrame(makeDroneCanMessageId(20, PM08_BATTERY_INFO_TYPE_ID, 99), 1);
    CanFrame actual{};

    TEST_ASSERT_TRUE(can.addFilter(filter));
    can.begin();

    TEST_ASSERT_FALSE(can.injectReceivedFrame(wrong_type));
    TEST_ASSERT_FALSE(can.injectReceivedFrame(service_frame));
    TEST_ASSERT_TRUE(can.injectReceivedFrame(matching_a));
    TEST_ASSERT_TRUE(can.injectReceivedFrame(matching_b));

    TEST_ASSERT_TRUE(can.receive(actual));
    TEST_ASSERT_EQUAL_MEMORY(&matching_a, &actual, sizeof(CanFrame));
    TEST_ASSERT_TRUE(can.receive(actual));
    TEST_ASSERT_EQUAL_MEMORY(&matching_b, &actual, sizeof(CanFrame));
}

void test_filters_cannot_be_modified_after_begin() {
    MockCan can;
    const CanFilter filter{
        static_cast<uint32_t>(PM08_NODE_STATUS_TYPE_ID) << 8,
        DRONECAN_MESSAGE_TYPE_MASK,
    };
    const CanFrame other = makeFrame(makeDroneCanMessageId(16, PM08_BATTERY_INFO_TYPE_ID, 42), 1);
    const CanFrame expected = makeFrame(makeDroneCanMessageId(16, PM08_NODE_STATUS_TYPE_ID, 42), 1);

    TEST_ASSERT_TRUE(can.addFilter(filter));
    can.begin();
    TEST_ASSERT_TRUE(can.started());

    TEST_ASSERT_FALSE(can.addFilter(filter));
    can.clearFilters();  // Ignored after begin(): filter stays active.
    TEST_ASSERT_FALSE(can.injectReceivedFrame(other));
    TEST_ASSERT_TRUE(can.injectReceivedFrame(expected));
}

void test_receive_rejects_oversized_payload() {
    MockCan can;
    CanFrame oversized = makeFrame(makeDroneCanMessageId(16, PM08_NODE_STATUS_TYPE_ID, 1), 0);
    oversized.length = CAN_MAX_DATA_LENGTH + 1;

    TEST_ASSERT_FALSE(can.injectReceivedFrame(oversized));
    TEST_ASSERT_FALSE(can.rxOverflowed());
}

void test_receive_queue_reports_overflow() {
    MockCan can;
    const CanFrame frame = makeFrame(makeDroneCanMessageId(16, PM08_NODE_STATUS_TYPE_ID, 1), 1);

    for (size_t index = 0; index < CAN_RX_QUEUE_CAPACITY; ++index) {
        TEST_ASSERT_TRUE(can.injectReceivedFrame(frame));
    }
    TEST_ASSERT_FALSE(can.injectReceivedFrame(frame));
    TEST_ASSERT_TRUE(can.rxOverflowed());

    can.clearRxOverflow();
    TEST_ASSERT_FALSE(can.rxOverflowed());
}

void test_send_records_frame_and_rejects_oversized_payload() {
    MockCan can;
    const CanFrame expected = makeFrame(makeDroneCanMessageId(16, PM08_BATTERY_INFO_TYPE_ID, 1), 4);
    CanFrame oversized = expected;
    oversized.length = CAN_MAX_DATA_LENGTH + 1;

    TEST_ASSERT_TRUE(can.send(expected));
    TEST_ASSERT_EQUAL_UINT(1, can.sentCount());
    TEST_ASSERT_EQUAL_MEMORY(&expected, &can.sentAt(0), sizeof(CanFrame));
    TEST_ASSERT_FALSE(can.send(oversized));
    TEST_ASSERT_EQUAL_UINT(1, can.sentCount());
}

void test_bus_state_is_reported() {
    MockCan can;

    can.setBusState(CanBusState::ErrorActive);
    TEST_ASSERT_EQUAL(CanBusState::ErrorActive, can.busState());
    can.setBusState(CanBusState::ErrorPassive);
    TEST_ASSERT_EQUAL(CanBusState::ErrorPassive, can.busState());
    can.setBusState(CanBusState::BusOff);  // ELE-05: CAN bus-off is a safety event.
    TEST_ASSERT_EQUAL(CanBusState::BusOff, can.busState());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_receive_is_non_blocking_when_queue_is_empty);
    RUN_TEST(test_received_frame_is_preserved);
    RUN_TEST(test_filter_matches_dronecan_type_id_with_mask);
    RUN_TEST(test_filters_cannot_be_modified_after_begin);
    RUN_TEST(test_receive_rejects_oversized_payload);
    RUN_TEST(test_receive_queue_reports_overflow);
    RUN_TEST(test_send_records_frame_and_rejects_oversized_payload);
    RUN_TEST(test_bus_state_is_reported);
    return UNITY_END();
}
