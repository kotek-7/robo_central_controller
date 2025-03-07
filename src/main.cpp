#include <unordered_map>
#include <m3508_control/core.hpp>
#include <bt_communication/core.hpp>
#include <can/core.hpp>
#include <can/peripheral.hpp>
#include <mpu6050_control/core.hpp>

/// @brief M3508へのCANの送信間隔(=PIDの制御周期)[ms]
constexpr uint32_t M3508_SEND_INTERVAL = 10;
/// @brief CAN通信の受信間隔(=PIDのFB受信間隔)間隔[ms]
constexpr uint32_t CAN_RECEIVE_INTERVAL = 10;
/// @brief シリアル通信の読み取り間隔[ms]
constexpr uint32_t SERIAL_READ_INTERVAL = 100;

/// @brief Bluetoothの受信イベントハンドラの登録を切り出した関数
void register_bt_event_handlers();
/// @brief CANの受信イベントハンドラの登録を切り出した関数
void register_can_event_handlers();

// std::make_unique()
// スマートポインタ(std::unique_ptr)を生成する。(ただのポインタよりメモリ安全)
// ヒープ上に各種インスタンスを生成することで、スタック領域の消費を抑える目論見だが、
// インスタンスがそんなに大きくないので普通にスタックに置いたほうが良いかも
// 各インスタンスの役割はクラスの定義を参照

auto bt_communicator = std::make_unique<bt_communication::BtCommunicator>();
auto can_communicator = std::make_unique<can::CanCommunicator>(*bt_communicator);   // 0x000のID受信用
auto m3508_controller = std::make_unique<m3508_control::M3508Controller>(*bt_communicator, *bt_communicator, *can_communicator);
auto mpu6050_controller = std::make_unique<mpu6050_control::Mpu6050Controller>();

void setup() {
    Serial.begin(115200);

    // MOSFETの電源をON
    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH);

    // 最上位層でtry-catchでエラーをキャッチすることで、エラーが発生した場合でもプログラムが停止しないようにする目論見
    // これでもなぜかときどきエラーでプログラム止まるのは謎
    try {
        bt_communicator->setup();
        can_communicator->setup();
        mpu6050_controller->setup();

        register_bt_event_handlers();
        register_can_event_handlers();
    } catch (const std::exception &e) {
        Serial.print("Unhandled error in setup: ");
        Serial.println(e.what());
        bt_communicator->remote_print("Unhandled error in setup: " + String(e.what()));
    }
}

void loop() {
    // カウンタとif文を使って、一定周期で処理を行う。
    // この方法はloop()内の関数がブロッキング動作(delay()など)を含まないように注意する必要がある。
    // また、内部で非同期処理(マルチスレッドの処理)を行う場合には注意が必要。(Bluetooth通信ライブラリが非同期処理してる気がするけど、大丈夫かな…？)
    // 処理が軽く、周期が超正確でなくてもいい場合には、順次処理が保証される上柔軟なのでいい感じ。

    delay(1);
    static uint32_t count = 0; // ループ回数(≈経過時間)
    count++;

    // setup()同様、try-catchでエラーをキャッチすることで、エラーが発生した場合でもプログラムが停止しないようにする目論見
    try {
        if (bt_communicator->is_device_connected()) {
            if (count % 100 == 0) {
                mpu6050_controller->remote_send_yaw();
            }
        }

        if (count % M3508_SEND_INTERVAL == 0) {
            m3508_controller->send_currents();
        }

        if (count % CAN_RECEIVE_INTERVAL == 0) {
            can_communicator->receive();
        }

        if (count % SERIAL_READ_INTERVAL == 0) {
            m3508_controller->read_serial_and_set_target_rpm();
        }

        if (count % 100 == 0) {
            float yaw = mpu6050_controller->get_yaw();
            Serial.println("yaw: " + String(yaw));
            bt_communicator->remote_print("yaw: " + String(yaw));
        }
    } catch (const std::exception &e) {
        Serial.print("Unhandled error in loop: ");
        Serial.println(e.what());
        bt_communicator->remote_print("Unhandled error in loop: " + String(e.what()));
    }
}

void register_bt_event_handlers() {
    // PIDゲインのセット(PID調整画面で動作)
    bt_communicator->add_write_event_listener("setPidGains", [&](JsonDocument doc) {
        Serial.println("pid gains set!");
        bt_communicator->remote_print("pid gains set!");

        m3508_controller->set_kp(doc["kp"].as<float>());
        m3508_controller->set_ki(doc["ki"].as<float>());
        m3508_controller->set_kd(doc["kd"].as<float>());
    });

    // 制御目標rpmのセット(PID調整画面で動作)
    bt_communicator->add_write_event_listener("setTargetRpm", [&](JsonDocument doc) {
        Serial.println("target rpm set!");
        bt_communicator->remote_print("target rpm set!");

        m3508_controller->set_target_rpm(doc["targetRpm"].as<float>());
    });

    // ジョイスティック入力をM3508の目標速度にセット
    bt_communicator->add_write_event_listener("joystick", [&](JsonDocument doc) {
        if (doc["side"] == "l") {
            constexpr float input_amp = 0.14;
            m3508_controller->set_target_velocity(
                Vec2(doc["leveledX"].as<float>(), doc["leveledY"].as<float>()) * input_amp
            );
        }
        if (doc["side"] == "r") {
            constexpr float input_amp = 15;
            m3508_controller->set_target_angular_velocity(doc["leveledX"].as<float>() * input_amp);
        }
    });

    // ボタン押下でコーン用ハンド0を閉める
    bt_communicator->add_write_event_listener("closeConeHand0", [&](JsonDocument doc) {
        Serial.println("command: closeConeHand0");
        bt_communicator->remote_print("command: closeConeHand0");
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_cone)
                .set_command(0x00)
                .build()
        );
    });

    // ボタン押下でコーン用ハンド0を開く
    bt_communicator->add_write_event_listener("openConeHand0", [&](JsonDocument doc) {
        Serial.println("command: openConeHand0");
        bt_communicator->remote_print("command: openConeHand0");
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_cone)
                .set_command(0x01)
                .build()
        );
    });

    // ボタン押下でコーン用ハンド1を閉める
    bt_communicator->add_write_event_listener("closeConeHand1", [&](JsonDocument doc) {
        Serial.println("command: closeConeHand1");
        bt_communicator->remote_print("command: closeConeHand1");
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_cone)
                .set_command(0x10)
                .build()
        );
    });

    // ボタン押下でコーン用ハンド1を開く
    bt_communicator->add_write_event_listener("openConeHand1", [&](JsonDocument doc) {
        Serial.println("command: openConeHand1");
        bt_communicator->remote_print("command: openConeHand1");
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_cone)
                .set_command(0x11)
                .build()
        );
    });

    // ボタン押下でボール用ハンドを閉める
    bt_communicator->add_write_event_listener("grabBall", [&](JsonDocument doc) {
        Serial.println("command: grabBall");
        bt_communicator->remote_print("command: grabBall");
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_ball)
                .set_command(0x00)
                .build()
        );
    });

    // ボタン押下でボール用ハンドを開く
    bt_communicator->add_write_event_listener("releaseBall", [&](JsonDocument doc) {
        Serial.println("command: releaseBall");
        bt_communicator->remote_print("command: releaseBall");
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_ball)
                .set_command(0x01)
                .build()
        );
    });

    // ボタン押下でボールを投げる
    bt_communicator->add_write_event_listener("throwBall", [&](JsonDocument doc) {
        Serial.println("command: throwBall");
        bt_communicator->remote_print("command: throwBall");
        can_communicator->transmit(
            can::CanTxMessageBuilder()
                .set_dest(can::CanDest::servo_ball)
                .set_command(0x02)
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
    /// TODO: 電力基盤からの電圧FBを受け取る
}