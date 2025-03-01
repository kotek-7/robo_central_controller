#pragma once

#include <Arduino.h>
#include "can/can_id.hpp"
#include "can_tx_message.hpp"

namespace can {
    class CanTransmitter {
    public:
        virtual ~CanTransmitter() = default;
        virtual void transmit(const CanTxMessage message) const = 0;
    };
} // namespace can