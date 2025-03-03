#pragma once

#include <Arduino.h>
#include "../values/can_tx_message.hpp"

namespace can {
    /// @brief CAN通信を送信するインターフェースです。
    class CanTransmitter {
    public:
        virtual ~CanTransmitter() = default;

        /// @brief CAN通信を送信します。
        /// @param message 送信するCANメッセージ
        virtual void transmit(const CanTxMessage message) const = 0;
    };
} // namespace can