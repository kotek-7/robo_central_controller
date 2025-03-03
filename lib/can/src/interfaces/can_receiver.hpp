#pragma once

#include <Arduino.h>
#include "../values/can_id.hpp"

namespace can {
    /// @brief CANを受信するインターフェースです。
    class CanReceiver {
    public:
        virtual ~CanReceiver() = default;

        /// @brief
        ///     CANを読み取ります。
        ///
        ///     CAN受信時のイベントハンドラも発火させます。
        virtual void receive() const = 0;

        /// @brief CAN受信時のイベントハンドラを追加します。
        /// @param listening_can_ids イベントハンドラを発火させるCAN IDのリスト
        /// @param listener 登録するイベントハンドラ
        virtual void add_reveive_event_listener(std::vector<can::CanId> listening_can_ids, std::function<void(const can::CanId, const std::array<uint8_t, 8>)> listener) = 0;
    };
} // namespace can