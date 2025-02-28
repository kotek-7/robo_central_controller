#include "can_tx_message_builder.hpp"

namespace can {
    CanTxMessageBuilder::CanTxMessageBuilder() :
        id(0), command(0), value({0, 0, 0, 0}), omake({0, 0, 0}) {}

    void CanTxMessageBuilder::set_id(const can::CanId id) {
        this->id = id;
    }

    void CanTxMessageBuilder::set_command(const uint8_t command) {
        this->command = command;
    }

    void CanTxMessageBuilder::set_value(const uint32_t value) {
        this->value[0] = (value >> 24) & 0xFF;
        this->value[1] = (value >> 16) & 0xFF;
        this->value[2] = (value >> 8) & 0xFF;
        this->value[3] = value & 0xFF;
    }

    void CanTxMessageBuilder::set_value(const float value) {
        this->set_value(*reinterpret_cast<const uint32_t *>(&value));
    }

    void CanTxMessageBuilder::set_omake(const std::array<uint8_t, 3> omake) {
        this->omake = omake;
    }

    void CanTxMessageBuilder::set_omake_0(const uint8_t omake) {
        this->omake[0] = omake;
    }

    void CanTxMessageBuilder::set_omake_1(const uint8_t omake) {
        this->omake[1] = omake;
    }

    void CanTxMessageBuilder::set_omake_2(const uint8_t omake) {
        this->omake[2] = omake;
    }

    CanTxMessage CanTxMessageBuilder::build() const {
        return CanTxMessage(id, {command, value[0], value[1], value[2], value[3], omake[0], omake[1], omake[2]});
    }
} // namespace can