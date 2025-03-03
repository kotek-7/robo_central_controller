#pragma once

#include "can_tx_message.hpp"
#include "can_dest.hpp"

namespace can {
    /// @brief CAN送信メッセージ(CanTxMessage)を構築するクラス
    /// @example
    ///     CanTxMessage message = CanTxMessageBuilder()
    ///         .set_dest(CanDest::servo_cone)
    ///         .set_command(0x10)
    ///         .set_value(100)
    ///         .build();
    ///     can_transmitter.transmit(message);
    class CanTxMessageBuilder {
    public:
        CanTxMessageBuilder();
        CanTxMessageBuilder &set_id(const can::CanId id);  // set_destを上書きします
        CanTxMessageBuilder &set_dest(const CanDest dest);  // set_idを上書きします
        CanTxMessageBuilder &set_command(const uint8_t command);
        CanTxMessageBuilder &set_value(const uint32_t value);
        CanTxMessageBuilder &set_value(const int32_t value);
        CanTxMessageBuilder &set_value(const float value, const float min_value, const float max_value);  // float型の値をint32_tの範囲にマッピングしてセット
        CanTxMessageBuilder &set_omake(const std::array<uint8_t, 3> omake);
        CanTxMessageBuilder &set_omake_0(const uint8_t omake);
        CanTxMessageBuilder &set_omake_1(const uint8_t omake);
        CanTxMessageBuilder &set_omake_2(const uint8_t omake);

        /// @brief CanTxMessageを構築します
        /// @return 
        CanTxMessage build() const;

    private:
        can::CanId id;
        uint8_t command;
        std::array<uint8_t, 4> value;
        std::array<uint8_t, 3> omake;
    };
} // namespace can
