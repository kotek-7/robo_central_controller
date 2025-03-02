#pragma once

#include <Arduino.h>
#include "../values/can_id.hpp"

namespace can {
    class CanReceiver {
    public:
        virtual ~CanReceiver() = default;
        virtual void receive() const = 0;
        virtual void add_reveive_event_listener(std::vector<can::CanId> can_ids, std::function<void(const can::CanId, const std::array<uint8_t, 8>)> listener) = 0;
    };
} // namespace can