#pragma once

#include "joystick_input.hpp"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

namespace bluetooth_communication {
    class BluetoothCommunicator {
    public:
        BluetoothCommunicator();
        void setup();
        void loop();
        /// @brief 左のジョイスティック入力を取得
        /// @return 左のジョイスティック入力
        joystick_input::JoystickInput get_joystick_l_input() const { return joystick_l_input; }
        /// @brief 右のジョイスティック入力を取得
        /// @return 右のジョイスティック入力
        joystick_input::JoystickInput get_joystick_r_input() const { return joystick_r_input; }

        void remote_print(String text);
        void remote_send_m3508_feedback(float angle, int16_t rpm, int16_t amp, uint8_t temp);
        void remote_send_m3508_pid_fields(float output, float p, float i, float d, float target_rpm, float error);

    private:
        /// @brief bluetoothデバイスが(1つ以上)接続されているか
        bool device_connected;
        /// @brief BLEサーバーへのポインタ
        BLEServer *p_server;
        /// @brief 送信用Characteristicへのポインタ
        BLECharacteristic *p_tx_characteristic;
        /// @brief 受信用Characteristicへのポインタ
        BLECharacteristic *p_rx_characteristic;
        /// @brief 左のジョイスティック入力
        joystick_input::JoystickInput joystick_l_input;
        /// @brief 右のジョイスティック入力
        joystick_input::JoystickInput joystick_r_input;

        void on_connect(BLEServer *p_server);
        void on_disconnect(BLEServer *p_server);
        void on_write(BLECharacteristic *p_characteristic);
        void remote_send_joystick_input(joystick_input::JoystickInput joystick_input, String side);
        joystick_input::JoystickInput parse_json_of_joystick_input(String json_string, String *side);
    };
} // namespace bluetooth_communication