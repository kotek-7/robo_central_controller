#include <Wire.h>
#include "mpu6050_controller.hpp"

namespace mpu6050_control {
    Mpu6050Controller::Mpu6050Controller() {}
    void Mpu6050Controller::setup() {
        Wire.begin();
        mpu6050.initialize();

        mpu6050.setXAccelOffset(0);
        mpu6050.setYAccelOffset(0);
        mpu6050.setZAccelOffset(0);
        mpu6050.setXGyroOffset(0);
        mpu6050.setYGyroOffset(0);
        mpu6050.setZGyroOffset(0);
    }

    void Mpu6050Controller::update() {
        const float angular_velocity = this->get_z_angular_velocity();
        const uint32_t dt = millis() - last_update;
        z_angle += angular_velocity * static_cast<float>(dt) / 1000.0;
    }

    float Mpu6050Controller::get_z_angular_velocity() {
        return static_cast<float>(mpu6050.getRotationZ()) * 250.0 / 32767.0;
    }
} // namespace mpu6050_control