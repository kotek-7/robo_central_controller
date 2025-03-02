#pragma once

#include <Arduino.h>
#include <vector>
#include <bt_communication/peripheral.hpp>
#include "../interfaces/can_transmitter.hpp"
#include "../interfaces/can_receiver.hpp"

namespace can {
    using CanId = uint32_t;

    class CanCommunicator : public CanTransmitter, public CanReceiver {
    public:
        CanCommunicator(bt_communication::BtInterface &bt_interface);

        void setup();
        void transmit(const CanTxMessage message) const override;
        void receive() const override;
        void add_reveive_event_listener(
            std::vector<can::CanId> can_ids, std::function<void(const can::CanId, const std::array<uint8_t, 8>)> listener
        ) override;

    private:
        std::vector<
            std::pair<
                std::vector<can::CanId>,
                std::function<void(const can::CanId, const std::array<uint8_t, 8>)>>>
            receive_event_listeners;
        bt_communication::BtInterface &bt_interface;
    };
} // namespace can