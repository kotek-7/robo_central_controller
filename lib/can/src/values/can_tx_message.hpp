#pragma once

#include "can_id.hpp"

namespace can {
    /// @brief
    ///     CAN送信メッセージ(CanTxMessage)を表すクラス
    ///
    ///     CanTxMessageBuilderで構築されることを想定しています。
    class CanTxMessage {
    public:
        CanTxMessage(can::CanId id, std::array<uint8_t, 8> data) :
            id(id), data(data) {};

        can::CanId id;
        std::array<uint8_t, 8> data;
    };
} // namespace can
