#pragma once

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <bt_communication/peripheral.hpp>

namespace mpu6050_control {
    /// @brief MPU6050から向きや加速度を読み取るクラス
    class Mpu6050Controller {
    public:
        Mpu6050Controller(const bt_communication::BtPrinter &bt_printer, const bt_communication::BtJsonSender &bt_json_sender);
        void setup();

        /// @brief ジャイロセンサからyaw角度を取得 [deg]
        float get_yaw();
        /// @brief ジャイロセンサからyaw角速度を取得 [deg/s]
        float get_yaw_velocity();
        /// @brief ジャイロセンサからyaw角度をBluetoothで送信
        void remote_send_yaw();

    private:
        MPU6050 mpu;
        uint32_t last_update;

        const bt_communication::BtPrinter &bt_printer;
        const bt_communication::BtJsonSender &bt_json_sender;
    };
} // namespace mpu6050_control