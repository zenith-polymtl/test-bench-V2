#include <stdint.h>
#include <unity.h>

#include "hal/ring_buffer.hpp"

using namespace hal;

void setUp() {}
void tearDown() {}

void test_empty_buffer_behavior() {
    RingBuffer<uint32_t, 4> buf;
    uint32_t value = 0;

    TEST_ASSERT_TRUE(buf.empty());
    TEST_ASSERT_EQUAL_UINT(0, buf.size());
    TEST_ASSERT_FALSE(buf.overflowed());
    TEST_ASSERT_FALSE(buf.pop(value));
}

void test_fifo_order_and_wraparound() {
    RingBuffer<uint32_t, 3> buf;
    uint32_t value = 0;

    TEST_ASSERT_TRUE(buf.push(10));
    TEST_ASSERT_TRUE(buf.push(20));
    TEST_ASSERT_FALSE(buf.empty());
    TEST_ASSERT_EQUAL_UINT(2, buf.size());

    TEST_ASSERT_TRUE(buf.pop(value));
    TEST_ASSERT_EQUAL_UINT32(10, value);
    TEST_ASSERT_EQUAL_UINT(1, buf.size());

    TEST_ASSERT_TRUE(buf.push(30));
    TEST_ASSERT_TRUE(buf.push(40));
    TEST_ASSERT_EQUAL_UINT(3, buf.size());

    TEST_ASSERT_TRUE(buf.pop(value));
    TEST_ASSERT_EQUAL_UINT32(20, value);
    TEST_ASSERT_TRUE(buf.pop(value));
    TEST_ASSERT_EQUAL_UINT32(30, value);
    TEST_ASSERT_TRUE(buf.pop(value));
    TEST_ASSERT_EQUAL_UINT32(40, value);
    TEST_ASSERT_TRUE(buf.empty());
    TEST_ASSERT_EQUAL_UINT(0, buf.size());
}

void test_overflow_flag_and_clear() {
    RingBuffer<uint32_t, 2> buf;
    uint32_t value = 0;

    TEST_ASSERT_TRUE(buf.push(1));
    TEST_ASSERT_TRUE(buf.push(2));
    TEST_ASSERT_FALSE(buf.push(3));
    TEST_ASSERT_TRUE(buf.overflowed());
    TEST_ASSERT_EQUAL_UINT(2, buf.size());

    buf.clearOverflow();
    TEST_ASSERT_FALSE(buf.overflowed());

    TEST_ASSERT_TRUE(buf.pop(value));
    TEST_ASSERT_EQUAL_UINT32(1, value);
    TEST_ASSERT_TRUE(buf.pop(value));
    TEST_ASSERT_EQUAL_UINT32(2, value);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_empty_buffer_behavior);
    RUN_TEST(test_fifo_order_and_wraparound);
    RUN_TEST(test_overflow_flag_and_clear);
    return UNITY_END();
}
