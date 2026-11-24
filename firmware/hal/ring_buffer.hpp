#pragma once

#include <stddef.h>

namespace hal {

template <typename T, size_t N>
class RingBuffer {
    static_assert(N > 0, "RingBuffer capacity must be greater than zero");

public:
    bool push(const T& item) {
        const size_t write = write_index_;
        const size_t next = (write + 1) % BUFFER_SLOTS;
        if (next == read_index_) {
            overflowed_ = true;
            return false;
        }
        buffer_[write] = item;
        write_index_ = next;
        return true;
    }

    bool pop(T& item) {
        const size_t read = read_index_;
        if (read == write_index_) return false;
        item = buffer_[read];
        read_index_ = (read + 1) % BUFFER_SLOTS;
        return true;
    }

    bool empty() const { return read_index_ == write_index_; }

    size_t size() const {
        const size_t write = write_index_;
        const size_t read = read_index_;
        return (write + BUFFER_SLOTS - read) % BUFFER_SLOTS;
    }

    bool overflowed() const { return overflowed_; }

    void clearOverflow() { overflowed_ = false; }

private:
    static constexpr size_t BUFFER_SLOTS = N + 1;

    T buffer_[BUFFER_SLOTS] = {};
    volatile size_t read_index_ = 0;
    volatile size_t write_index_ = 0;
    volatile bool overflowed_ = false;
};

}  // namespace hal
