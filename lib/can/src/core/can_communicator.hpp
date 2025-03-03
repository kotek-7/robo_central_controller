#pragma once

#include <Arduino.h>
#include <vector>
#include <bt_communication/peripheral.hpp>
#include "../interfaces/can_transmitter.hpp"
#include "../interfaces/can_receiver.hpp"

namespace can {
    using CanId = uint32_t;

    /// @brief CAN通信を行うクラス
    /// @details
    ///     CAN通信を行うクラスです。
    ///     CAN通信の送信と受信を行うためのCanTransmitterとCanReceiverを継承しています。
    ///     使う前にはsetup()関数を呼び出して初期化する必要があります。
    class CanCommunicator : public CanTransmitter, public CanReceiver {
    public:
        CanCommunicator(bt_communication::BtInterface &bt_interface);

        /// @brief セットアップ処理。使う前に呼び出す！
        void setup();
        void transmit(const CanTxMessage message) const override;
        void receive() const override;
        void add_reveive_event_listener(
            std::vector<can::CanId> listening_can_ids, std::function<void(const can::CanId, const std::array<uint8_t, 8>)> listener
        ) override;

    private:
        /// @brief CAN受信時のイベントリスナのリスト
        std::vector<
            std::pair<
                std::vector<can::CanId>,
                std::function<void(const can::CanId, const std::array<uint8_t, 8>)>>>
            receive_event_listeners;

        /// @brief Bluetooth通信のインターフェース
        bt_communication::BtInterface &bt_interface;
    };
} // namespace can