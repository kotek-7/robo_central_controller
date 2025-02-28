#pragma once

#include <Arduino.h>
#include "can/can_id.hpp"

namespace can {
    class CanTransmitter {
    public:
        virtual ~CanTransmitter() = default;
        virtual void transmit(const can::CanId rx_id, const uint8_t tx_buf[]) const = 0;
    };
} // namespace can