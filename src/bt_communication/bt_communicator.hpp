#pragma once

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <ArduinoJson.h>
#include <memory>
#include "m3508_control/c620_id.hpp"
#include "joystick_input.hpp"

namespace bt_communication {
    /// @brief Bluetooth通信を行うクラス
    /// @details
    ///     このクラスは、コントローラ・モニタとのBLE(Bluetooth Low Energy)通信を行います。
    ///
    ///     BLEでコントローラからのジョイスティック入力を自動で受け取り、保持します。
    ///     また、モニタにBLEで各種データを送信するための関数も提供します。
    ///     Bluetooth通信の受信時のイベントハンドラを登録することもできます。
    ///
    ///     使う前にsetup()を呼び出して初期化する必要があります。
    class BtCommunicator {
    public:
        BtCommunicator();
        void setup();

        /// @brief Bluetoothデバイスが(1つ以上)接続されているか
        bool is_device_connected() const { return device_connected; }
        /// @brief 左のジョイスティック入力を取得
        joystick_input::JoystickInput get_joystick_l_input() const { return joystick_l_input; }
        /// @brief 右のジョイスティック入力を取得
        joystick_input::JoystickInput get_joystick_r_input() const { return joystick_r_input; }

        void remote_print(String text);
        void
        remote_send_m3508_feedback(m3508_control::C620Id c620_id, float angle, int16_t rpm, int16_t amp, uint8_t temp);
        void remote_send_m3508_pid_fields(
            m3508_control::C620Id c620_id, float output, float p, float i, float d, float target_rpm, float error
        );
        void add_write_event_listener(String type, std::function<void(JsonDocument doc)> listener);

    private:
        /// @brief bluetoothデバイスが(1つ以上)接続されているか
        bool device_connected;
        /// @brief BLEサーバーへのポインタ
        std::unique_ptr<BLEServer> ble_server;
        /// @brief 送信用Characteristicへのポインタ
        std::unique_ptr<BLECharacteristic> tx_characteristic;
        /// @brief 受信用Characteristicへのポインタ
        std::unique_ptr<BLECharacteristic> rx_characteristic;
        /// @brief 左のジョイスティック入力
        joystick_input::JoystickInput joystick_l_input;
        /// @brief 右のジョイスティック入力
        joystick_input::JoystickInput joystick_r_input;
        std::vector<std::pair<String, std::function<void(JsonDocument doc)>>> on_write_event_listeners;

        void on_connect(BLEServer *server);
        void on_disconnect(BLEServer *server);
        void on_write(BLECharacteristic *characteristic);
        void remote_send_joystick_input(joystick_input::JoystickInput joystick_input, String side);
        void set_and_forward_joystick_input(JsonDocument doc);
    };
} // namespace bt_communication