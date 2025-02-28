#pragma once

#include <Arduino.h>
#include "utils/types.hpp"

namespace can {
    class CanReceiver {
    public:
        virtual ~CanReceiver() = default;
        virtual void receive() = 0;
        virtual void add_reveive_event_listener(std::function<void(const utils::CanId, const std::array<uint8_t, 8>)> listener) = 0;
    };
} // namespace can