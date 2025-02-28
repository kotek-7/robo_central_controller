#pragma once

#include "can_tx_message.hpp"

namespace can
{
    class CanTxMessageBuilder
    {
    public:
        CanTxMessageBuilder();
        CanTxMessageBuilder& set_id(const can::CanId id);
        CanTxMessageBuilder& set_command(const uint8_t command);
        CanTxMessageBuilder& set_value(const uint32_t value);
        CanTxMessageBuilder& set_value(const int32_t value);
        CanTxMessageBuilder& set_value(const float value, const float min_value, const float max_value);
        CanTxMessageBuilder& set_omake(const std::array<uint8_t, 3> omake);
        CanTxMessageBuilder& set_omake_0(const uint8_t omake);
        CanTxMessageBuilder& set_omake_1(const uint8_t omake);
        CanTxMessageBuilder& set_omake_2(const uint8_t omake);

        CanTxMessage build() const;
    private:
        can::CanId id;
        uint8_t command;
        std::array<uint8_t, 4> value;
        std::array<uint8_t, 3> omake;
    };
} // namespace can
