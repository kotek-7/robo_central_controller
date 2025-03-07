#include <Arduino.h>
#include <Wire.h>
#include "mpu6050_controller.hpp"

namespace mpu6050_control {
    Mpu6050Controller::Mpu6050Controller() :
        last_update(millis()) {}
    void Mpu6050Controller::setup() {
        Wire.begin();
        Wire.setClock(400000);
        mpu.initialize();

        auto test_connection_result = mpu.testConnection();
        if (test_connection_result) {
            Serial.println("MPU6050 connection successful");
        } else {
            Serial.println("MPU6050 connection failed");
        }

        mpu.dmpInitialize();

        mpu.setXAccelOffset(0);
        mpu.setYAccelOffset(0);
        mpu.setZAccelOffset(0);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);

        Serial.println("Calibrating gyro and accel..");
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println("Calibration done.");

        mpu.setDMPEnabled(true);
        Serial.println("DMP enabled.");
    }

    float Mpu6050Controller::get_yaw() {
        uint8_t fifo_buffer[64];
        mpu.dmpGetCurrentFIFOPacket(fifo_buffer);

        Quaternion q;
        mpu.dmpGetQuaternion(&q, fifo_buffer);

        VectorFloat gravity;
        mpu.dmpGetGravity(&gravity, &q);

        float yaw_pitch_roll[3];
        mpu.dmpGetYawPitchRoll(yaw_pitch_roll, &q, &gravity);

        return yaw_pitch_roll[0] * 180.0 / M_PI;
    }
} // namespace mpu6050_control