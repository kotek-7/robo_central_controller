#pragma once

#include <MPU6050.h>

namespace mpu6050_control {
    /// @brief MPU6050から向きや加速度を読み取るクラス
    class Mpu6050Controller {
    public:
        Mpu6050Controller();
        void setup();

        /// @brief 角度の積分計算を行う なるべく早い周期で呼び出してほしい
        void update();
        float get_z_angle() const { return this->z_angle; };
        float get_z_angular_velocity();
    private:
        MPU6050 mpu6050;
        float z_angle = 0;
        uint32_t last_update;
    };
} // namespace mpu6050_control