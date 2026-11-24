#include "hal/teensy/teensy_can.hpp"

namespace hal {

TeensyCan* TeensyCan::active_instance_ = nullptr;

void TeensyCan::begin() {
    active_instance_ = this;
    can_.begin();
    can_.setBaudRate(CAN_BITRATE_BPS);
    can_.enableFIFO();
    can_.setFIFOFilter(REJECT_ALL);
    if (filter_count_ == 0) {
        can_.setFIFOUserFilter(0, 0, 0, EXT);
    } else {
        for (size_t index = 0; index < filter_count_; ++index) {
            can_.setFIFOUserFilter(static_cast<uint8_t>(index),
                                   filters_[index].id,
                                   filters_[index].mask,
                                   EXT);
        }
    }
    can_.onReceive(receiveCallback);
    can_.enableFIFOInterrupt();
    started_ = true;
}

bool TeensyCan::send(const CanFrame& frame) {
    if (frame.length > CAN_MAX_DATA_LENGTH) return false;
    CAN_message_t message = {};
    message.id = frame.id;
    message.len = frame.length;
    message.flags.extended = 1;
    for (uint8_t index = 0; index < frame.length; ++index) message.buf[index] = frame.data[index];
    return can_.write(message);
}

bool TeensyCan::receive(CanFrame& frame) {
    noInterrupts();
    const bool ok = rx_buffer_.pop(frame);
    interrupts();
    return ok;
}

bool TeensyCan::addFilter(const CanFilter& filter) {
    if (started_ || filter_count_ >= CAN_MAX_FILTERS) return false;
    filters_[filter_count_++] = filter;
    return true;
}

void TeensyCan::clearFilters() {
    if (started_) return;
    filter_count_ = 0;
}

CanBusState TeensyCan::busState() const {
    // FlexCAN ESR1 FLTCONF: 0 = active, 1 = passive, 2 = bus-off.
    const uint32_t fault_state = (FLEXCAN1_ESR1 >> 4) & 0x03U;
    if (fault_state == 2U) return CanBusState::BusOff;
    if (fault_state == 1U) return CanBusState::ErrorPassive;
    return CanBusState::ErrorActive;
}

void TeensyCan::clearRxOverflow() {
    noInterrupts();
    rx_buffer_.clearOverflow();
    interrupts();
}

void TeensyCan::receiveCallback(const CAN_message_t& message) {
    if (active_instance_ == nullptr) return;
    if (!message.flags.extended || message.len > CAN_MAX_DATA_LENGTH) return;
    CanFrame frame;
    frame.id = message.id;
    frame.length = message.len;
    frame.received_at_us = micros();
    for (uint8_t index = 0; index < frame.length; ++index) frame.data[index] = message.buf[index];
    active_instance_->rx_buffer_.push(frame);
}

}  // namespace hal