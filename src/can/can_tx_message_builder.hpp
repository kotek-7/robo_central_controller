#pragma once

#include "can_tx_message.hpp"

namespace can
{
    class CanTxMessageBuilder
    {
    public:
        CanTxMessageBuilder();
        void set_id(const can::CanId id);
        void set_command(const uint8_t command);
        void set_value(const uint32_t value);
        void set_value(const float value);
        void set_omake(const std::array<uint8_t, 3> omake);
        void set_omake_0(const uint8_t omake);
        void set_omake_1(const uint8_t omake);
        void set_omake_2(const uint8_t omake);

        CanTxMessage build() const;
    private:
        can::CanId id;
        uint8_t command;
        std::array<uint8_t, 4> value;
        std::array<uint8_t, 3> omake;
    };
} // namespace can
