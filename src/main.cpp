#include <unordered_map>
#include "m3508_control/m3508_controller.hpp"
#include "bt_communication/bt_communicator.hpp"
#include "can/can_communicator.hpp"
#include "can/can_tx_message_builder.hpp"

constexpr uint32_t CAN_SEND_INTERVAL = 20;
constexpr uint32_t CAN_RECEIVE_INTERVAL = 20;
constexpr uint32_t SERIAL_READ_INTERVAL = 100;

void register_bt_event_handlers();
void register_can_event_handlers();

/// Bluetooth通信クラス
std::unique_ptr<bt_communication::BtCommunicator> bt_communicator(new bt_communication::BtCommunicator());
///  Bluetoothでいろいろやり取りする関数をまとめたクラス
std::unique_ptr<bt_communication::BtInterface> bt_interface(new bt_communication::BtInterface(
    [](String text) { bt_communicator->remote_print(text); },
    [](m3508_control::C620Id c620_id, float angle, int16_t rpm, int16_t amp, uint8_t temp) {
        bt_communicator->remote_send_m3508_feedback(c620_id, angle, rpm, amp, temp);
    },
    [](m3508_control::C620Id c620_id, float output, float p, float i, float d, float target_rpm, float error) {
        bt_communicator->remote_send_m3508_pid_fields(c620_id, output, p, i, d, target_rpm, error);
    }
));
/// CAN通信クラス
std::unique_ptr<can::CanCommunicator> can_communicator(new can::CanCommunicator(*bt_interface));
/// M3508モータの制御クラス
std::unique_ptr<m3508_control::M3508Controller> m3508_controller(new m3508_control::M3508Controller(*bt_interface, *can_communicator));

void setup() {
    Serial.begin(115200);
    randomSeed(0); // 乱数生成器のシード値を設定(高速化のため)
    try {
        bt_communicator->setup();
        can_communicator->setup();
        m3508_controller->setup();

        register_bt_event_handlers();
        register_can_event_handlers();
    } catch (const std::exception &e) {
        Serial.print("Unhandled error in setup: ");
        Serial.println(e.what());
        bt_communicator->remote_print("Unhandled error in setup: " + String(e.what()));
    }
}

void loop() {
    static uint32_t count = 0;
    count++;

    try {
        if (bt_communicator->is_device_connected()) {
            // Bluetooth接続時の処理
        }

        // 制御量をPIDで計算してM3508に送信
        if (count % CAN_SEND_INTERVAL == 0) {
            m3508_controller->send_currents();
        }

        // M3508からのフィードバック値を読み取り
        if (count % CAN_RECEIVE_INTERVAL == 0) {
            can_communicator->receive();
        }

        // シリアル通信で制御目標値を読み取って設定
        if (count % SERIAL_READ_INTERVAL == 0) {
            m3508_controller->read_serial_and_set_target_rpm();
        }
    } catch (const std::exception &e) {
        Serial.print("Unhandled error in loop: ");
        Serial.println(e.what());
        bt_communicator->remote_print("Unhandled error in loop: " + String(e.what()));
    }

    delay(1);
}

void register_bt_event_handlers() {
    // Bluetooth通信の受信時のイベントハンドラとしてPIDゲインをセットする処理を追加
    bt_communicator->add_write_event_listener("setPidGains", [&](JsonDocument doc) {
        m3508_controller->set_kp(doc["kp"].as<float>());
        m3508_controller->set_ki(doc["ki"].as<float>());
        m3508_controller->set_kd(doc["kd"].as<float>());
    });
    // Bluetooth通信の受信時のイベントハンドラとして制御目標値をセットする処理を追加
    bt_communicator->add_write_event_listener("setTargetRpm", [&](JsonDocument doc) {
        m3508_controller->set_target_rpm(doc["targetRpm"].as<float>());
    });

    bt_communicator->add_write_event_listener("joystick", [&](JsonDocument doc) {
        constexpr float input_amp = 0.05;
        if (doc["side"] != "l") {
            return;
        }
        m3508_controller->set_target_velocity(
            utils::Vec2(doc["leveledX"].as<float>(), doc["leveledY"].as<float>()) * input_amp
        );
    });

    bt_communicator->add_write_event_listener("closeConeHand0", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_cone)
                .set_command(0x00)
                .build()
        );
    });

    bt_communicator->add_write_event_listener("openConeHand0", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_cone)
                .set_command(0x01)
                .build()
        );
    });

    bt_communicator->add_write_event_listener("closeConeHand0", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_cone)
                .set_command(0x10)
                .build()
        );
    });

    bt_communicator->add_write_event_listener("openConeHand0", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_cone)
                .set_command(0x11)
                .build()
        );
    });

    bt_communicator->add_write_event_listener("grabBall", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_ball)
                .set_command(0x00)
                .build()
        );
    });

    bt_communicator->add_write_event_listener("releaseBall", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_ball)
                .set_command(0x01)
                .build()
        );
    });

    bt_communicator->add_write_event_listener("throwBall", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_ball)
                .set_command(0x02)
                .build()
        );
    });

    bt_communicator->add_write_event_listener("moveConeHand0", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::dc_0)
                .set_command(0x00)
                .set_value(doc["value"].as<float>(), -100.0f, 100.0f)
                .build()
        );
    });

    bt_communicator->add_write_event_listener("moveConeHand1", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::dc_1)
                .set_command(0x00)
                .set_value(doc["value"].as<float>(), -100.0f, 100.0f)
                .build()
        );
    });

    bt_communicator->add_write_event_listener("moveBallHand", [&](JsonDocument doc) {
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::dc_2)
                .set_command(0x00)
                .set_value(doc["value"].as<float>(), -100.0f, 100.0f)
                .build()
        );
    });
}

void register_can_event_handlers() {
    // CAN通信の受信時のイベントハンドラとしてM3508のフィードバック値をBluetoothで送信する処理を追加
    can_communicator->add_reveive_event_listener(
        {0x201, 0x202, 0x203, 0x204},
        [&](const can::CanId rx_id, const std::array<uint8_t, 8> rx_buf) {
            m3508_controller->set_feedback(rx_id, rx_buf);
        }
    );
}