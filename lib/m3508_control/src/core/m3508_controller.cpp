#include <driver/twai.h>
#include "m3508_controller.hpp"

// モータの制御処理
namespace m3508_control {
    // M3508モータのCAN送信ID
    constexpr uint16_t CAN_ID = 0x200;

    // PID制御用定数
    constexpr float KP = 0.7;
    constexpr float KI = 0.0005;
    constexpr float KD = 50;
    constexpr float CLAMPING_OUTPUT = 5000; // 電流値のクランピング値 [mA]

    constexpr uint8_t FEEDBACK_SEND_INTERVAL = 10;

    M3508Controller::M3508Controller(
        const bt_communication::BtJsonSender &bt_json_sender,
        const bt_communication::BtPrinter &bt_printer,
        const can::CanTransmitter &can_transmitter
    ) :
        pid_controllers({
            {C620Id::C1,
             pid_controller::PIDController(
                 KP,
                 KI,
                 KD,
                 CLAMPING_OUTPUT,
                 bt_printer,
                 [&](float output, float proportional, float integral, float derivative, float target_rpm, float error) {
                     JsonDocument doc;
                     doc["type"] = "m3508PidFields";
                     doc["c620Id"] = static_cast<uint8_t>(C620Id::C1);
                     doc["output"] = output;
                     doc["p"] = proportional;
                     doc["i"] = integral;
                     doc["d"] = derivative;
                     doc["targetRpm"] = target_rpm;
                     doc["error"] = error;
                     bt_json_sender.remote_send_json(doc);
                 }
             )},
            {C620Id::C2,
             pid_controller::PIDController(
                 KP,
                 KI,
                 KD,
                 CLAMPING_OUTPUT,
                 bt_printer,
                 [&](float output, float proportional, float integral, float derivative, float target_rpm, float error
                 ) {
                     JsonDocument doc;
                     doc["type"] = "m3508PidFields";
                     doc["c620Id"] = static_cast<uint8_t>(C620Id::C2);
                     doc["output"] = output;
                     doc["p"] = proportional;
                     doc["i"] = integral;
                     doc["d"] = derivative;
                     doc["targetRpm"] = target_rpm;
                     doc["error"] = error;
                     bt_json_sender.remote_send_json(doc);
                 }
             )},
            {C620Id::C3,
             pid_controller::PIDController(
                 KP,
                 KI,
                 KD,
                 CLAMPING_OUTPUT,
                 bt_printer,
                 [&](float output, float proportional, float integral, float derivative, float target_rpm, float error
                 ) {
                     JsonDocument doc;
                     doc["type"] = "m3508PidFields";
                     doc["c620Id"] = static_cast<uint8_t>(C620Id::C3);
                     doc["output"] = output;
                     doc["p"] = proportional;
                     doc["i"] = integral;
                     doc["d"] = derivative;
                     doc["targetRpm"] = target_rpm;
                     doc["error"] = error;
                     bt_json_sender.remote_send_json(doc);
                 }
             )},
            {C620Id::C4,
             pid_controller::PIDController(
                 KP,
                 KI,
                 KD,
                 CLAMPING_OUTPUT,
                 bt_printer,
                 [&](float output, float proportional, float integral, float derivative, float target_rpm, float error
                 ) {
                     JsonDocument doc;
                     doc["type"] = "m3508PidFields";
                     doc["c620Id"] = static_cast<uint8_t>(C620Id::C4);
                     doc["output"] = output;
                     doc["p"] = proportional;
                     doc["i"] = integral;
                     doc["d"] = derivative;
                     doc["targetRpm"] = target_rpm;
                     doc["error"] = error;
                     bt_json_sender.remote_send_json(doc);
                 }
             )},
        }),
        angular_velocity_pid_controller(3, 0, 0, 250, bt_printer),
        target_velocity(Vec2(0, 0)),
        bt_json_sender(bt_json_sender),
        bt_printer(bt_printer),
        can_transmitter(can_transmitter) {}

    void M3508Controller::send_currents() {
        update_target_rpms();
        int32_t command_currents[4];
        for (auto &pid_controller : pid_controllers) {
            command_currents[static_cast<uint8_t>(pid_controller.first) - 1] = pid_controller.second.update_output();
        }
        std::array<uint8_t, 8> tx_buf;
        milli_amperes_to_bytes(command_currents, tx_buf.data());

        can_transmitter.transmit(can::CanTxMessage(CAN_ID, tx_buf));
    }

    void M3508Controller::set_feedback(const can::CanId rx_id, const std::array<uint8_t, 8> rx_buf) {
        C620Id rx_c620_id = static_cast<C620Id>(rx_id - 0x200);

        float angle;
        int16_t rpm;
        int16_t amp;
        uint8_t temp;
        derive_feedback_fields(rx_buf.data(), &angle, &rpm, &amp, &temp);
        pid_controllers.at(rx_c620_id).set_feedback_values(angle, rpm, amp, temp);

        static uint32_t count = 0;
        if (count % FEEDBACK_SEND_INTERVAL == 0) {
            remote_send_feedback(rx_c620_id, angle, rpm, amp, temp);
        }
    }

    void M3508Controller::read_serial_and_set_target_rpm() {
        if (Serial.available()) {
            delay(1); // 一連のシリアル信号をすべて受信するまで待つ
            String input_string = "";
            while (Serial.available() > 0) {
                char input_char = Serial.read();
                input_string.concat(input_char);
            }
            pid_controllers.at(C620Id::C1).set_target_rpm(input_string.toInt());
            pid_controllers.at(C620Id::C2).set_target_rpm(input_string.toInt());
            pid_controllers.at(C620Id::C3).set_target_rpm(input_string.toInt());
            pid_controllers.at(C620Id::C4).set_target_rpm(input_string.toInt());
            Serial.print("Set target rpm to: " + input_string);
            Serial.print("\n\n");

            bt_printer.remote_print("Set target rpm to: " + input_string);
        }
    }

    void M3508Controller::set_kp(float p) {
        pid_controllers.at(C620Id::C1).set_kp(p);
        pid_controllers.at(C620Id::C2).set_kp(p);
        pid_controllers.at(C620Id::C3).set_kp(p);
        pid_controllers.at(C620Id::C4).set_kp(p);
    };

    void M3508Controller::set_ki(float i) {
        pid_controllers.at(C620Id::C1).set_ki(i);
        pid_controllers.at(C620Id::C2).set_ki(i);
        pid_controllers.at(C620Id::C3).set_ki(i);
        pid_controllers.at(C620Id::C4).set_ki(i);
    };

    void M3508Controller::set_kd(float d) {
        pid_controllers.at(C620Id::C1).set_kd(d);
        pid_controllers.at(C620Id::C2).set_kd(d);
        pid_controllers.at(C620Id::C3).set_kd(d);
        pid_controllers.at(C620Id::C4).set_kd(d);
    };

    void M3508Controller::set_target_rpm(float target_rpm) {
        pid_controllers.at(C620Id::C1).set_target_rpm(target_rpm);
        pid_controllers.at(C620Id::C2).set_target_rpm(target_rpm);
        pid_controllers.at(C620Id::C3).set_target_rpm(target_rpm);
        pid_controllers.at(C620Id::C4).set_target_rpm(target_rpm);
    };

    void M3508Controller::set_target_velocity(const Vec2 &target_velocity) {
        this->target_velocity = target_velocity;
        update_target_rpms();
    }

    void M3508Controller::set_target_angular_velocity(const float target_angular_velocity) {
        angular_velocity_pid_controller.set_target_value(target_angular_velocity);
    }

    void M3508Controller::set_yaw(float yaw) {
        this->yaw = yaw;
    }

    void M3508Controller::set_yaw_velocity(float yaw_velocity) {
        angular_velocity_pid_controller.set_feedback_value(yaw_velocity);
    }

    void M3508Controller::update_target_rpms() {
        float command_angular_velocity = angular_velocity_pid_controller.update_output();
        float target_rpm_1, target_rpm_2, target_rpm_3, target_rpm_4;
        calc_target_rpms(target_velocity, command_angular_velocity, yaw, &target_rpm_1, &target_rpm_2, &target_rpm_3, &target_rpm_4);
        pid_controllers.at(C620Id::C1).set_target_rpm(target_rpm_1);
        pid_controllers.at(C620Id::C2).set_target_rpm(target_rpm_2);
        pid_controllers.at(C620Id::C3).set_target_rpm(target_rpm_3);
        pid_controllers.at(C620Id::C4).set_target_rpm(target_rpm_4);
    }

    void M3508Controller::remote_send_feedback(
        const C620Id c620_id, const float angle, const int16_t rpm, const int16_t amp, const uint8_t temp
    ) const {
        JsonDocument doc;
        doc["type"] = "m3508Feedback";
        doc["c620Id"] = static_cast<uint8_t>(c620_id);
        doc["angle"] = angle;
        doc["rpm"] = rpm;
        doc["amp"] = amp;
        doc["temp"] = temp;
        bt_json_sender.remote_send_json(doc);
    }

    void M3508Controller::milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8]) const {
        uint8_t i;
        for (i = 0; i < 4; i++) {
            int32_t milli_ampere = milli_amperes[i] * 16384 / 20000;
            uint8_t upper = (milli_ampere >> 8) & 0xFF;
            uint8_t lower = milli_ampere & 0xFF;
            out_tx_buf[i * 2] = upper;
            out_tx_buf[i * 2 + 1] = lower;
        }
    }

    void M3508Controller::derive_feedback_fields(
        const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp
    ) const {
        *out_angle = (float)(rx_buf[0] << 8 | rx_buf[1]) * 360.0f / 8191.0f;
        *out_rpm = rx_buf[2] << 8 | rx_buf[3];
        *out_amp = rx_buf[4] << 8 | rx_buf[5];
        *out_temp = rx_buf[6];
    }

    void M3508Controller::calc_target_rpms(
        const Vec2 &target_velocity,
        const float target_angular_velocity,
        const float current_yaw,
        float *out_target_rpm_1,
        float *out_target_rpm_2,
        float *out_target_rpm_3,
        float *out_target_rpm_4
    ) const {
        constexpr float one_over_root_2 = 0.70710678118f; // 1.0 / sqrt(2.0)
        constexpr float wheel_radius = 0.051f;            // 車輪の半径(m)
        constexpr float reduction_ratio = 19.2032085561;  // 減速比
        constexpr float robot_radius = 0.231f;            // ロボットの中心からホイールまでの距離(m)

        const Vec2 rotated_target_velocity = target_velocity.rotate(current_yaw);

        *out_target_rpm_1 = (one_over_root_2 * (rotated_target_velocity.x - rotated_target_velocity.y) - robot_radius * target_angular_velocity / 180 * M_PI)
                            / wheel_radius * 60.0f / (2.0f * M_PI) * reduction_ratio;
        *out_target_rpm_2 = (one_over_root_2 * (-rotated_target_velocity.x - rotated_target_velocity.y) - robot_radius * target_angular_velocity / 180 * M_PI)
                            / wheel_radius * 60.0f / (2.0f * M_PI) * reduction_ratio;
        *out_target_rpm_3 = (one_over_root_2 * (-rotated_target_velocity.x + rotated_target_velocity.y) - robot_radius * target_angular_velocity / 180 * M_PI)
                            / wheel_radius * 60.0f / (2.0f * M_PI) * reduction_ratio;
        *out_target_rpm_4 = (one_over_root_2 * (rotated_target_velocity.x + rotated_target_velocity.y) - robot_radius * target_angular_velocity / 180 * M_PI)
                            / wheel_radius * 60.0f / (2.0f * M_PI) * reduction_ratio;
    }
} // namespace m3508_control