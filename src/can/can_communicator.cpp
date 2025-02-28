#include "can_communicator.hpp"
#include "driver/twai.h"

namespace can {
    constexpr gpio_num_t CAN_TX = GPIO_NUM_16;
    constexpr gpio_num_t CAN_RX = GPIO_NUM_4;

    CanCommunicator::CanCommunicator(bt_communication::BtInterface &bt_interface) :
        receive_event_listeners(), bt_interface(bt_interface) {}

    void CanCommunicator::setup() {
        twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
        twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
        twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        const auto driver_install_result = twai_driver_install(&general_config, &timing_config, &filter_config);
        if (driver_install_result == ESP_OK) {
            Serial.println("Driver install OK!");
            bt_interface.remote_print("Driver install OK!");
        } else {
            Serial.println("Driver install fail!");
            bt_interface.remote_print("Driver install fail!");
            return;
        }

        if (twai_start() == ESP_OK) {
            Serial.println("Driver start OK!");
            bt_interface.remote_print("Driver start OK!");
        } else {
            Serial.println("Driver start fail!");
            bt_interface.remote_print("Driver start fail!");
            return;
        }
    }

    void CanCommunicator::transmit(const utils::CanId tx_id, const uint8_t tx_buf[8]) const {
        twai_message_t tx_message;
        tx_message.identifier = tx_id;
        tx_message.extd = 0;
        tx_message.rtr = 0;
        tx_message.ss = 0;
        tx_message.self = 0;
        tx_message.dlc_non_comp = 0;
        tx_message.data_length_code = 8;
        for (uint8_t i = 0; i < 8; i++) {
            tx_message.data[i] = tx_buf[i];
        }

        const auto tx_result = twai_transmit(&tx_message, 0);
        if (tx_result != ESP_OK) {
            Serial.println("Transmit Fail: The TX queue is full!");
            bt_interface.remote_print("Transmit Fail: The TX queue is full!");
        }
    }

    void CanCommunicator::receive() const {
        twai_message_t rx_message;
        const auto rx_result = twai_receive(&rx_message, 0);
        if (rx_result != ESP_OK) {
            Serial.println("Receive Fail: The RX queue is empty!");
            bt_interface.remote_print("Receive Fail: The RX queue is empty!");
        }
        if (rx_message.rtr) {
            Serial.println("Receive Fail: The received message is a remote frame!");
            bt_interface.remote_print("Receive Fail: The received message is a remote frame!");
            return;
        }
        if (rx_message.extd) {
            Serial.println("Receive Fail: The received message is an extended frame!");
            bt_interface.remote_print("Receive Fail: The received message is an extended frame!");
            return;
        }

        const auto rx_id = rx_message.identifier;
        std::array<uint8_t, 8> rx_buf = {};
        for (uint8_t i = 0; i < 8; i++) {
            rx_buf[i] = rx_message.data[i];
        }

        for (const auto &listener : receive_event_listeners) {
            if (std::any_of(listener.first.begin(), listener.first.end(), [&rx_id](const utils::CanId &can_id) { return can_id == rx_id; })) {
                listener.second(rx_id, rx_buf);
            }
        }
    }

    void CanCommunicator::add_reveive_event_listener(
        std::vector<utils::CanId> can_ids,
        std::function<void(const utils::CanId, const std::array<uint8_t, 8>)> listener
    ) {
        receive_event_listeners.push_back(std::make_pair(can_ids, listener));
    }
} // namespace can