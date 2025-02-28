#pragma once

#include <Arduino.h>
#include <vector>
#include "can_transmitter.hpp"
#include "can_receiver.hpp"
#include "bt_communication/bt_interface.hpp"

namespace can {
    using CanId = uint32_t;

    class CanCommunicator : public CanTransmitter, public CanReceiver {
    public:
        CanCommunicator(bt_communication::BtInterface &bt_interface);

        void setup();
        void transmit(const utils::CanId tx_id, const uint8_t tx_buf[8]) const override;
        void receive() const override;
        void add_reveive_event_listener(
            std::function<void(const utils::CanId, const std::array<uint8_t, 8>)> listener
        ) override;

    private:
        std::vector<std::function<void(const utils::CanId, const std::array<uint8_t, 8>)>> receive_event_listeners;
        bt_communication::BtInterface &bt_interface;
    };
} // namespace can