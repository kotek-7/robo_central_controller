#pragma once

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

namespace mpu6050_control {
    /// @brief MPU6050から向きや加速度を読み取るクラス
    class Mpu6050Controller {
    public:
        Mpu6050Controller();
        void setup();

        float get_yaw();
    private:
        MPU6050 mpu;
        uint32_t last_update;
    };
} // namespace mpu6050_control