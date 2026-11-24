#pragma once

#include <FlexCAN_T4.h>

#include "hal/can.hpp"
#include "hal/ring_buffer.hpp"

namespace hal {

class TeensyCan : public ICan {
public:
    void begin() override;
    bool send(const CanFrame& frame) override;
    bool receive(CanFrame& frame) override;
    bool addFilter(const CanFilter& filter) override;
    void clearFilters() override;
    CanBusState busState() const override;
    // Tracks software ring-buffer overflows only; FlexCAN_T4 clears the 6-frame hardware FIFO
    // overflow flag internally before user code can inspect it.
    bool rxOverflowed() const override { return rx_buffer_.overflowed(); }
    void clearRxOverflow() override;

private:
    static void receiveCallback(const CAN_message_t& message);

    static TeensyCan* active_instance_;
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_;
    RingBuffer<CanFrame, CAN_RX_QUEUE_CAPACITY> rx_buffer_;
    // Staged until begin(), because FlexCAN hardware filters cannot be programmed before enableFIFO().
    CanFilter filters_[CAN_MAX_FILTERS] = {};
    size_t filter_count_ = 0;
    bool started_ = false;
};

}  // namespace hal