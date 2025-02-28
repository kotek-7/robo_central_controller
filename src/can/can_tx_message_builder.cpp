#include <cstring>
#include "can_tx_message_builder.hpp"

namespace can {
    CanTxMessageBuilder::CanTxMessageBuilder() :
        id(0), command(0), value({0, 0, 0, 0}), omake({0, 0, 0}) {}

    CanTxMessageBuilder& CanTxMessageBuilder::set_id(const can::CanId id) {
        this->id = id;
        return *this;
    }

    CanTxMessageBuilder& CanTxMessageBuilder::set_command(const uint8_t command) {
        this->command = command;
        return *this;
    }

    CanTxMessageBuilder& CanTxMessageBuilder::set_value(const uint32_t value) {
        this->value[0] = (value >> 24) & 0xFF;
        this->value[1] = (value >> 16) & 0xFF;
        this->value[2] = (value >> 8) & 0xFF;
        this->value[3] = value & 0xFF;
        return *this;
    }

    CanTxMessageBuilder& CanTxMessageBuilder::set_value(const int32_t value) {
        this->value[0] = (value >> 24) & 0xFF;
        this->value[1] = (value >> 16) & 0xFF;
        this->value[2] = (value >> 8) & 0xFF;
        this->value[3] = value & 0xFF;
        return *this;
    }

    CanTxMessageBuilder& CanTxMessageBuilder::set_value(const float value, const float min_value, const float max_value) {
        constexpr int32_t max_int32 = 2147483647;
        constexpr int32_t min_int32 = -2147483648;

        const auto normalized_value = (value - min_value) / (max_value - min_value);
        const auto scaled_value = static_cast<int32_t>(0xFFFFFFFF * normalized_value + min_int32);
        const auto clamped_value = std::max(min_int32, std::min(max_int32, scaled_value));  // オーバーフローしない気はするけど一応の対策です

        return this->set_value(clamped_value);
    }

    CanTxMessageBuilder& CanTxMessageBuilder::set_omake(const std::array<uint8_t, 3> omake) {
        this->omake = omake;
        return *this;
    }

    CanTxMessageBuilder& CanTxMessageBuilder::set_omake_0(const uint8_t omake) {
        this->omake[0] = omake;
        return *this;
    }

    CanTxMessageBuilder& CanTxMessageBuilder::set_omake_1(const uint8_t omake) {
        this->omake[1] = omake;
        return *this;
    }

    CanTxMessageBuilder& CanTxMessageBuilder::set_omake_2(const uint8_t omake) {
        this->omake[2] = omake;
        return *this;
    }

    CanTxMessage CanTxMessageBuilder::build() const {
        return CanTxMessage(id, {command, value[0], value[1], value[2], value[3], omake[0], omake[1], omake[2]});
    }
} // namespace can