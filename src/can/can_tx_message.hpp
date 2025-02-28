#pragma once

#include "can/can_id.hpp"

namespace can {
    class CanTxMessage {
    public:
        CanTxMessage(can::CanId id, std::array<uint8_t, 8> data);

    private:
        can::CanId id;
        std::array<uint8_t, 8> data;
    };

    CanTxMessage::CanTxMessage(can::CanId id, std::array<uint8_t, 8> data) :
        id(id), data(data) {}
} // namespace can
