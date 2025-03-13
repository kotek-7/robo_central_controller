#include <Arduino.h>
#include <Wire.h>
#include "mpu6050_controller.hpp"

namespace mpu6050_control {
    Mpu6050Controller::Mpu6050Controller(const bt_communication::BtPrinter &bt_printer, const bt_communication::BtJsonSender &bt_json_sender) :
        last_update(millis()),
        yaw_offset(0),
        bt_printer(bt_printer),
        bt_json_sender(bt_json_sender) {}

    void Mpu6050Controller::setup() {
        mpu.reset();
        Wire.begin();
        Wire.setClock(400000);
        mpu.initialize();

        auto test_connection_result = mpu.testConnection();
        if (test_connection_result) {
            Serial.println("MPU6050 connection successful");
            bt_printer.remote_print("MPU6050 connection successful");
        } else {
            Serial.println("MPU6050 connection failed");
            bt_printer.remote_print("MPU6050 connection failed");
        }

        mpu.dmpInitialize();

        mpu.setXAccelOffset(0);
        mpu.setYAccelOffset(0);
        mpu.setZAccelOffset(0);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);

        Serial.println("Calibrating gyro and accel..");
        bt_printer.remote_print("Calibrating gyro and accel..");
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println("Calibration done.");
        bt_printer.remote_print("Calibration done.");

        mpu.setDMPEnabled(true);
        Serial.println("DMP enabled.");
        bt_printer.remote_print("DMP enabled.");
    }

    float Mpu6050Controller::get_yaw() {
        return get_raw_yaw() + yaw_offset;
    }

    float Mpu6050Controller::get_yaw_velocity() {
        float yaw_velocity = static_cast<float>(mpu.getRotationZ()) / 16.4; // ジャイロの感度スケールファクタに基づく
        return yaw_velocity;
    }

    void Mpu6050Controller::reset_yaw() {
        yaw_offset = -get_raw_yaw();
    }

    void Mpu6050Controller::remote_send_yaw() {
        float yaw = get_yaw();

        JsonDocument doc;
        doc["type"] = "yaw";
        doc["value"] = yaw;
        bt_json_sender.remote_send_json(doc);
    }

    float Mpu6050Controller::get_raw_yaw() {
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