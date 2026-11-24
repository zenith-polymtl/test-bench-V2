#pragma once

#include "hal/can.hpp"
#include "hal/ring_buffer.hpp"

namespace hal {

class MockCan : public ICan {
public:
    void begin() override { started_ = true; }

    bool send(const CanFrame& frame) override {
        if (frame.length > CAN_MAX_DATA_LENGTH) return false;
        if (sent_count_ >= CAN_RX_QUEUE_CAPACITY) return false;
        sent_[sent_count_++] = frame;
        return true;
    }

    bool receive(CanFrame& frame) override { return rx_buffer_.pop(frame); }

    bool addFilter(const CanFilter& filter) override {
        if (started_ || filter_count_ >= CAN_MAX_FILTERS) return false;
        filters_[filter_count_++] = filter;
        return true;
    }

    void clearFilters() override {
        if (started_) return;
        filter_count_ = 0;
    }

    CanBusState busState() const override { return bus_state_; }
    bool rxOverflowed() const override { return rx_buffer_.overflowed(); }
    void clearRxOverflow() override { rx_buffer_.clearOverflow(); }

    bool injectReceivedFrame(const CanFrame& frame) {
        if (frame.length > CAN_MAX_DATA_LENGTH || !accepted(frame)) return false;
        return rx_buffer_.push(frame);
    }

    void setBusState(CanBusState state) { bus_state_ = state; }
    bool started() const { return started_; }
    size_t sentCount() const { return sent_count_; }
    const CanFrame& sentAt(size_t index) const { return sent_[index]; }

private:
    bool accepted(const CanFrame& frame) const {
        if (filter_count_ == 0) return true;
        for (size_t index = 0; index < filter_count_; ++index) {
            const uint32_t mask = filters_[index].mask;
            if ((frame.id & mask) == (filters_[index].id & mask)) return true;
        }
        return false;
    }

    RingBuffer<CanFrame, CAN_RX_QUEUE_CAPACITY> rx_buffer_;
    CanFrame sent_[CAN_RX_QUEUE_CAPACITY] = {};
    CanFilter filters_[CAN_MAX_FILTERS] = {};
    size_t sent_count_ = 0;
    size_t filter_count_ = 0;
    CanBusState bus_state_ = CanBusState::ErrorActive;
    bool started_ = false;
};

}  // namespace hal
