#pragma once

#include "can/values/can_id.hpp"

namespace can {
    class CanTxMessage {
    public:
        CanTxMessage(can::CanId id, std::array<uint8_t, 8> data) :
            id(id), data(data) {};

        can::CanId id;
        std::array<uint8_t, 8> data;
    };
} // namespace can
