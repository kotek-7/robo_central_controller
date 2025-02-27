#include <unordered_map>
#include "m3508_control/m3508_controller.hpp"
#include "bt_communication/bt_communicator.hpp"

constexpr uint32_t CAN_SEND_INTERVAL = 20;
constexpr uint32_t CAN_RECEIVE_INTERVAL = 20;
constexpr uint32_t SERIAL_READ_INTERVAL = 100;

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
/// M3508モータの制御クラス
std::unique_ptr<m3508_control::M3508Controller> m3508_controller(new m3508_control::M3508Controller(*bt_interface));

void setup() {
    Serial.begin(115200);
    randomSeed(0); // 乱数生成器のシード値を設定(高速化のため)
    try {
        bt_communicator->setup();
        m3508_controller->setup();

        // Bluetooth通信の受信時のイベントハンドラとしてPIDゲインをセットする処理を追加
        bt_communicator->add_write_event_listener([&](JsonDocument doc) {
            if (doc["type"] != "setPidGains") {
                return;
            }
            m3508_controller->set_kp(doc["kp"].as<float>());
            m3508_controller->set_ki(doc["ki"].as<float>());
            m3508_controller->set_kd(doc["kd"].as<float>());
        });
        // Bluetooth通信の受信時のイベントハンドラとして制御目標値をセットする処理を追加
        bt_communicator->add_write_event_listener([&](JsonDocument doc) {
            if (doc["type"] != "setTargetRpm") {
                return;
            }
            m3508_controller->set_target_rpm(doc["targetRpm"].as<float>());
        });

        bt_communicator->add_write_event_listener([&](JsonDocument doc) {
            constexpr float input_amp = 0.05;
            if (doc["type"] != "joystick") {
                return;
            }
            if (doc["side"] != "l") {
                return;
            }
            m3508_controller->set_target_velocity(
                utils::Vec2(doc["leveledX"].as<float>(), doc["leveledY"].as<float>()) * input_amp
            );
        });
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
            m3508_controller->read_and_set_feedback();
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