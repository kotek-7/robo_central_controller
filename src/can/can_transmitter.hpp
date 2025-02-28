#pragma once

#include <Arduino.h>
#include "utils/types.hpp"

namespace can {
    class CanTransmitter {
    public:
        virtual ~CanTransmitter() = default;
        virtual void transmit(const utils::CanId rx_id, const uint8_t tx_buf[]) = 0;
    };
} // namespace can