#include "can_communicator.hpp"
#include "driver/twai.h"

namespace can {
    constexpr gpio_num_t CAN_TX = GPIO_NUM_16;
    constexpr gpio_num_t CAN_RX = GPIO_NUM_4;

    CanCommunicator::CanCommunicator(bt_communication::BtPrinter &bt_printer) :
        receive_event_listeners(), bt_printer(bt_printer) {}

    void CanCommunicator::setup() {
        twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
        twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
        twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        const auto driver_install_result = twai_driver_install(&general_config, &timing_config, &filter_config);
        if (driver_install_result == ESP_OK) {
            Serial.println("Driver install OK!");
            bt_printer.remote_print("Driver install OK!");
        } else {
            Serial.println("Driver install fail!");
            bt_printer.remote_print("Driver install fail!");
            return;
        }

        if (twai_start() == ESP_OK) {
            Serial.println("Driver start OK!");
            bt_printer.remote_print("Driver start OK!");
        } else {
            Serial.println("Driver start fail!");
            bt_printer.remote_print("Driver start fail!");
            return;
        }
    }

    void CanCommunicator::transmit(const CanTxMessage message) const {
        twai_message_t tx_message;
        tx_message.identifier = message.id;
        tx_message.extd = 0;
        tx_message.rtr = 0;
        tx_message.ss = 0;
        tx_message.self = 0;
        tx_message.dlc_non_comp = 0;
        tx_message.data_length_code = 8;
        for (uint8_t i = 0; i < 8; i++) {
            tx_message.data[i] = message.data[i];
        }

        const auto tx_result = twai_transmit(&tx_message, 0);
        if (tx_result == ESP_ERR_TIMEOUT) {
            Serial.println("Transmit Fail: ESP_ERR_TIMEOUT");
            bt_printer.remote_print("Transmit Fail: ESP_ERR_TIMEOUT");
            return;
        }
        if (tx_result == ESP_ERR_INVALID_ARG) {
            Serial.println("Transmit Fail: ESP_ERR_INVALID_ARG");
            bt_printer.remote_print("Transmit Fail: ESP_ERR_INVALID_ARG");
            return;
        }
        if (tx_result == ESP_ERR_INVALID_STATE) {
            Serial.println("Transmit Fail: ESP_ERR_INVALID_STATE");
            bt_printer.remote_print("Transmit Fail: ESP_ERR_INVALID_STATE");
            return;
        }
        if (tx_result == ESP_FAIL) {
            Serial.println("Transmit Fail: ESP_FAIL");
            bt_printer.remote_print("Transmit Fail: ESP_FAIL");
            return;
        }
        if (tx_result == ESP_ERR_NOT_SUPPORTED) {
            Serial.println("Transmit Fail: ESP_ERR_NOT_SUPPORTED");
            bt_printer.remote_print("Transmit Fail: ESP_ERR_NOT_SUPPORTED");
            return;
        }
        if (tx_result != ESP_OK) {
            Serial.println("Transmit Fail: Unexpected error: " + String(tx_result));
            bt_printer.remote_print("Transmit Fail: Unexpected error: " + String(tx_result));
            return;
        }
    }

    void CanCommunicator::receive() const {
        twai_message_t rx_message;
        const auto rx_result = twai_receive(&rx_message, 0);
        if (rx_result == ESP_ERR_TIMEOUT) {
            Serial.println("Receive fail: ESP_ERR_TIMEOUT");
            bt_printer.remote_print("Receive fail: ESP_ERR_TIMEOUT");
            return;
        }
        if (rx_result == ESP_ERR_INVALID_ARG) {
            Serial.println("Receive fail: ESP_ERR_INVALID_ARG");
            bt_printer.remote_print("Receive fail: ESP_ERR_INVALID_ARG");
            return;
        }
        if (rx_result == ESP_ERR_INVALID_STATE) {
            Serial.println("Receive fail: ESP_ERR_INVALID_STATE");
            bt_printer.remote_print("Receive fail: ESP_ERR_INVALID_STATE");
            return;
        }
        if (rx_result != ESP_OK) {
            Serial.println("Receive fail: Unexpected error: " + String(rx_result));
            bt_printer.remote_print("Receive fail: Unexpected error: " + String(rx_result));
            return;
        }
        if (rx_message.rtr) {
            Serial.println("Receive Fail: The received message is a remote frame!");
            bt_printer.remote_print("Receive Fail: The received message is a remote frame!");
            return;
        }
        if (rx_message.extd) {
            Serial.println("Receive Fail: The received message is an extended frame!");
            bt_printer.remote_print("Receive Fail: The received message is an extended frame!");
            return;
        }

        const auto rx_id = rx_message.identifier;
        std::array<uint8_t, 8> rx_buf = {};
        for (uint8_t i = 0; i < 8; i++) {
            rx_buf[i] = rx_message.data[i];
        }

        for (const auto &listener : receive_event_listeners) {
            if (std::any_of(listener.first.begin(), listener.first.end(), [&rx_id](const can::CanId &can_id) { return can_id == rx_id; })) {
                listener.second(rx_id, rx_buf);
            }
        }
    }

    void CanCommunicator::add_reveive_event_listener(
        std::vector<can::CanId> listening_can_ids,
        std::function<void(const can::CanId, const std::array<uint8_t, 8>)> listener
    ) {
        receive_event_listeners.push_back(std::make_pair(listening_can_ids, listener));
    }
} // namespace can