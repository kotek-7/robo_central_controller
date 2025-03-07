// https://qiita.com/takudooon/items/2ab77f22196504ff9597
// https://qiita.com/umi_kappa/items/dd3d7a27cf714971406e

#include <vector>
#include "bt_communicator.hpp"

// コントローラーとの通信処理
// Bluetooth(BLE)通信の概要: https://www.musen-connect.co.jp/blog/course/trial-production/ble-beginner-1/
namespace bt_communication {
    // uuid生成: https://www.uuidgenerator.net/
    constexpr const char *SERVICE_UUID = "0a133f79-efe1-40c5-b4a5-cba5980d0d0f";
    constexpr const char *TX_CHARACTERISTIC_UUID = "7687561d-1dba-458f-9fb2-58e6b85208ef";
    constexpr const char *RX_CHARACTERISTIC_UUID = "8c83ffae-8421-4581-9755-10c5efd53d10";

    /// @brief ジョイスティック入力をモニターに転送する間隔(ループ回数)
    constexpr const u_int8_t JOYSTICK_INPUT_FORWARD_INTERVAL = 1000;

    BtCommunicator::BtCommunicator() :
        device_connected(false),
        ble_server(nullptr),
        tx_characteristic(nullptr),
        rx_characteristic(nullptr),
        joystick_l_input(joystick_input::JoystickInput()),
        joystick_r_input(joystick_input::JoystickInput()) {}

    /// @brief オブジェクトを使う前に呼び出す！(主にBLEの初期化)
    void BtCommunicator::setup() {
        // 通信時の各種コールバッククラスの作成
        class ServerCallbacks : public BLEServerCallbacks {
        public:
            BtCommunicator &bt_communicator;

            ServerCallbacks(BtCommunicator &bt_communicator) :
                bt_communicator(bt_communicator) {}
            void onConnect(BLEServer *server) override { bt_communicator.on_connect(server); }
            void onDisconnect(BLEServer *server) override { bt_communicator.on_disconnect(server); }
        };
        class TxCharacteristicCallbacks : public BLECharacteristicCallbacks {};
        class RxCharacteristicCallbacks : public BLECharacteristicCallbacks {
        public:
            BtCommunicator &bt_communicator;

            RxCharacteristicCallbacks(BtCommunicator &bt_communicator) :
                bt_communicator(bt_communicator) {}
            void onWrite(BLECharacteristic *characteristic) override { bt_communicator.on_write(characteristic); }
        };

        // BLEの初期化
        BLEDevice::init("esp32_for_BLE");
        ble_server = std::unique_ptr<BLEServer>(BLEDevice::createServer());
        ble_server->setCallbacks(new ServerCallbacks(*this));

        // Serviceを作成
        BLEService *p_service = ble_server->createService(SERVICE_UUID);

        // 送信用Characteristicを作成
        {
            tx_characteristic = std::unique_ptr<BLECharacteristic>(p_service->createCharacteristic(
                TX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
            ));
            tx_characteristic->setCallbacks(new TxCharacteristicCallbacks());

            // BLE2902: Client Characteristc Configuration Descriptor
            tx_characteristic->addDescriptor(new BLE2902());
        }

        // 受信用Characteristicを作成
        {
            rx_characteristic = std::unique_ptr<BLECharacteristic>(
                p_service->createCharacteristic(RX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE)
            );
            rx_characteristic->setCallbacks(new RxCharacteristicCallbacks(*this));
        }

        // Bluetooth受信時にジョイスティック入力を読み取りモニターに転送するイベントリスナを登録
        add_write_event_listener("joystick", [this](JsonDocument doc) { this->set_and_forward_joystick_input(doc); });

        // 通信開始
        p_service->start();
        BLEAdvertising *p_advertising = ble_server->getAdvertising();
        p_advertising->start();
    }

    /// @brief bluetooth通信の接続時の処理(主にデバッグログの出力)
    /// @param server BLEサーバを一時的に参照するためのポインタ
    void BtCommunicator::on_connect(BLEServer *server) {
        Serial.println("connected!");
        remote_print("conncted!");

        // 接続中のデバイスの数を表示
        auto connected_devices = server->getPeerDevices(false); // この引数falseは内部で無視されている
        Serial.print("connected devices: ");
        Serial.println(String(connected_devices.size()));
        remote_print("connected devices: " + String(connected_devices.size()));

        delay(500);
        server->startAdvertising(); // アドバタイズを再開して、更に複数のセントラルとの接続を受付
        Serial.println("restart advertising..");
        remote_print("restart advertising..");
        device_connected = true;
    }

    /// @brief bluetooth通信の切断時の処理(主にデバッグログの出力)
    /// @param server BLEサーバを一時的に参照するためのポインタ
    void BtCommunicator::on_disconnect(BLEServer *server) {
        Serial.println("disconnected!");
        remote_print("disconnected!");

        // 接続中のデバイスの数を表示
        auto connected_devices = server->getPeerDevices(false); // この引数falseは内部で無視されている
        Serial.print("connected devices: ");
        Serial.println(String(connected_devices.size()));
        remote_print("connected devices: " + String(connected_devices.size()));

        delay(500);
        server->startAdvertising();
        Serial.println("start advertising..");
        remote_print("start advertising..");
        device_connected = false;
    }

    /// @brief bluetooth通信の受信時の処理
    /// @details 受信したデータをデシリアライズし、受信したデータを処理するイベントリスナを呼び出します。
    /// @param characteristic 通信を受信したCharacteristicを一時的に参照するためのポインタ
    void BtCommunicator::on_write(BLECharacteristic *characteristic) {
        // BLEで受信したデータを取得
        String rx_buf = String(characteristic->getValue().c_str());

        // 受信したjsonのデータをデシリアライズ
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, rx_buf);
        if (error) {
            Serial.print("deserialization error: ");
            Serial.print(error.c_str());
        }

        // 受信したデータを処理するイベントリスナを全部呼び出す
        for (auto listener : on_write_event_listeners) {
            if (listener.first == doc["type"]) {
                listener.second(doc);
            }
        }
    }

    void BtCommunicator::remote_send_json(JsonDocument doc) const {
        if (tx_characteristic == nullptr) {
            Serial.println("error: tx_characteristic is null");
            return;
        }

        String tx_json_string;
        serializeJson(doc, tx_json_string);
        tx_characteristic->setValue(tx_json_string.c_str());
        tx_characteristic->notify();
    }

    /// @brief モニターのコンソールにテキストを送信
    void BtCommunicator::remote_print(const String text) const {
        JsonDocument doc;
        doc["type"] = "print";
        doc["text"] = text;
        remote_send_json(doc);
    }

    /// @brief モニターにモータのフィードバック値を送信
    void BtCommunicator::remote_send_m3508_feedback(
        m3508_control::C620Id c620_id, float angle, int16_t rpm, int16_t amp, uint8_t temp
    ) {
        JsonDocument doc;
        doc["type"] = "m3508Feedback";
        doc["c620Id"] = static_cast<uint8_t>(c620_id);
        doc["angle"] = angle;
        doc["rpm"] = rpm;
        doc["amp"] = amp;
        doc["temp"] = temp;
        remote_send_json(doc);
    }

    /// @brief モニターにモータのpid制御値を送信
    void BtCommunicator::remote_send_m3508_pid_fields(
        m3508_control::C620Id c620_id, float output, float p, float i, float d, float target_rpm, float error
    ) {
        JsonDocument doc;
        doc["type"] = "m3508PidFields";
        doc["c620Id"] = static_cast<uint8_t>(c620_id);
        doc["output"] = output;
        doc["p"] = p;
        doc["i"] = i;
        doc["d"] = d;
        doc["targetRpm"] = target_rpm;
        doc["error"] = error;
        remote_send_json(doc);
    }

    /// @brief BLEで受信したデータを処理するリスナーを追加
    void BtCommunicator::add_write_event_listener(String type, std::function<void(JsonDocument doc)> listener) {
        on_write_event_listeners.push_back(std::make_pair(type, listener));
    }

    /// @brief モニターにジョイスティックの入力値を転送
    void BtCommunicator::remote_send_joystick_input(joystick_input::JoystickInput joystick_input, String side) {
        JsonDocument doc;
        doc["type"] = "joystickInput";
        doc["side"] = side;
        doc["x"] = joystick_input.get_input()->x;
        doc["y"] = joystick_input.get_input()->y;
        doc["leveledX"] = joystick_input.get_leveled_input()->x;
        doc["leveledY"] = joystick_input.get_leveled_input()->y;
        doc["distance"] = joystick_input.get_distance();
        doc["angle"] = joystick_input.get_angle();
        remote_send_json(doc);
    }

    /// @brief ジョイスティックの入力値をメンバ変数に格納し、モニターに転送
    void BtCommunicator::set_and_forward_joystick_input(JsonDocument doc) {
        static uint32_t count = 0;
        count++;

        // ジョイスティックの入力値を取得して、メンバ変数に格納
        if (doc["side"] == "l") {
            auto input = Vec2(doc["x"], doc["y"]);
            auto leveled_input = Vec2(doc["leveledX"], doc["leveledY"]);
            float distance = doc["distance"];
            float angle = doc["angle"];
            joystick_l_input = joystick_input::JoystickInput(input, leveled_input, distance, angle);
        } else if (doc["side"] == "r") {
            auto input = Vec2(doc["x"], doc["y"]);
            auto leveled_input = Vec2(doc["leveledX"], doc["leveledY"]);
            float distance = doc["distance"];
            float angle = doc["angle"];
            joystick_r_input = joystick_input::JoystickInput(input, leveled_input, distance, angle);
        } else {
            Serial.println("error: invalid side");
            remote_print("error: invalid side");
            return;
        }

        // ジョイスティックの入力をモニターに転送(バッファの圧迫を防ぐため、一定間隔で)
        if (count % JOYSTICK_INPUT_FORWARD_INTERVAL == 0) {
            remote_send_joystick_input(joystick_l_input, String("l"));
            remote_send_joystick_input(joystick_r_input, String("r"));
        }
    }
} // namespace bt_communication