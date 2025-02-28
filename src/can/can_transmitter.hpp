#pragma once

#include <Arduino.h>
#include "can/can_id.hpp"

namespace can {
    class CanTransmitter {
    public:
        virtual ~CanTransmitter() = default;
        virtual void transmit(const can::CanId rx_id, const std::array<uint8_t, 8> tx_buf) const = 0;
    };
} // namespace can