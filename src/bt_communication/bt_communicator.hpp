#pragma once

#include "joystick_input.hpp"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

namespace bt_communication {
    /// @brief Bluetooth通信を行うクラス
    /// @details
    ///     このクラスは、コントローラ・モニタとのBluetooth通信を行います。
    ///     main.cpp内でsetup()関数を呼び出すことでBluetoothの初期化を行い、
    ///     loop()関数を呼び出すことでBluetooth関連のループ処理を行います。
    ///     コントローラのジョイスティック入力を取得し、保持します。
    ///     また、モニタにデータを送信するための関数も提供します。
    ///     BLE(Bluetooth Low Energy)の
    class BtCommunicator {
    public:
        BtCommunicator();
        ~BtCommunicator();
        void setup();
        void loop();

        /// @brief Bluetoothデバイスが(1つ以上)接続されているか
        bool is_device_connected() const { return device_connected; }
        /// @brief 左のジョイスティック入力を取得
        joystick_input::JoystickInput get_joystick_l_input() const { return joystick_l_input; }
        /// @brief 右のジョイスティック入力を取得
        joystick_input::JoystickInput get_joystick_r_input() const { return joystick_r_input; }

        void remote_print(String text);
        void remote_send_m3508_feedback(float angle, int16_t rpm, int16_t amp, uint8_t temp);
        void remote_send_m3508_pid_fields(float output, float p, float i, float d, float target_rpm, float error);

    private:
        /// @brief bluetoothデバイスが(1つ以上)接続されているか
        bool device_connected;
        /// @brief BLEサーバーへのポインタ
        BLEServer *ble_server;
        /// @brief 送信用Characteristicへのポインタ
        BLECharacteristic *tx_characteristic;
        /// @brief 受信用Characteristicへのポインタ
        BLECharacteristic *rx_characteristic;
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
} // namespace bt_communication