
// https://qiita.com/takudooon/items/2ab77f22196504ff9597
// https://qiita.com/umi_kappa/items/dd3d7a27cf714971406e

#include "bluetooth_communicator.hpp"
#include "joystick_input.hpp"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// コントローラーとの通信処理
namespace bluetooth_communication {
    // uuid生成: https://www.uuidgenerator.net/
    constexpr const char *SERVICE_UUID = "0a133f79-efe1-40c5-b4a5-cba5980d0d0f";
    constexpr const char *TX_CHARACTERISTIC_UUID = "7687561d-1dba-458f-9fb2-58e6b85208ef";
    constexpr const char *RX_CHARACTERISTIC_UUID = "8c83ffae-8421-4581-9755-10c5efd53d10";

    constexpr const u_int8_t JOYSTICK_RESEND_RATE = 10;

    BluetoothCommunicator::BluetoothCommunicator()
        : device_connected(false),
          p_server(nullptr),
          p_tx_characteristic(nullptr),
          p_rx_characteristic(nullptr),
          joystick_l_input(joystick_input::JoystickInput()),
          joystick_r_input(joystick_input::JoystickInput()) {}

    /// @brief セットアップ
    void BluetoothCommunicator::setup() {
        class ServerCallbacks : public BLEServerCallbacks {
        public:
            BluetoothCommunicator *p_bluetooth_communicator;

            ServerCallbacks(BluetoothCommunicator *p_bluetooth_communicator)
                : p_bluetooth_communicator(p_bluetooth_communicator) {}
            void onConnect(BLEServer *pServer) override { p_bluetooth_communicator->onConnect(pServer); }
            void onDisconnect(BLEServer *pServer) override { p_bluetooth_communicator->onDisconnect(pServer); }
        };

        class TxCharacteristicCallbacks : public BLECharacteristicCallbacks {};

        class RxCharacteristicCallbacks : public BLECharacteristicCallbacks {
        public:
            BluetoothCommunicator *p_bluetooth_communicator;

            RxCharacteristicCallbacks(BluetoothCommunicator *p_bluetooth_communicator)
                : p_bluetooth_communicator(p_bluetooth_communicator) {}
            void onWrite(BLECharacteristic *pCharacteristic) override {
                p_bluetooth_communicator->onWrite(pCharacteristic);
            }
        };

        BLEDevice::init("esp32_for_BLE");
        p_server = BLEDevice::createServer();
        p_server->setCallbacks(new ServerCallbacks(this));

        // Serviceを作成
        BLEService *p_service = p_server->createService(SERVICE_UUID);

        // 送信用Characteristicを作成
        {
            p_tx_characteristic = p_service->createCharacteristic(
                TX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
            );
            p_tx_characteristic->setCallbacks(new TxCharacteristicCallbacks());

            // Client Characteristc Configuration Descriptor
            BLE2902 *pCccd = new BLE2902();
            p_tx_characteristic->addDescriptor(pCccd);
        }

        // 受信用Characteristicを作成
        {
            p_rx_characteristic =
                p_service->createCharacteristic(RX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
            p_rx_characteristic->setCallbacks(new RxCharacteristicCallbacks(this));
        }

        // 通信開始
        p_service->start();
        BLEAdvertising *p_advertising = p_server->getAdvertising();
        p_advertising->start();
    }

    /// @brief メインループ
    void BluetoothCommunicator::loop() {
        // 接続中
        if (device_connected) {
            int random_num = random(255);
            remote_print("[sample output] random num: ");
            remote_print(String(random_num));
            Serial.println("[sample output] random num: ");
            Serial.println(String(random_num));
            delay(2000);
        }
        delay(10);
    }

    /// @brief bluetooth通信の接続時
    /// @param p_server BLEサーバへのポインタ
    void BluetoothCommunicator::onConnect(BLEServer *p_server) {
        Serial.println("connected!");
        remote_print("conncted!");

        // 接続中のデバイスの数を表示
        auto connected_devices = p_server->getPeerDevices(false); // この引数falseは内部で無視されている
        Serial.print("connected devices: ");
        Serial.println(String(connected_devices.size()));
        remote_print("connected devices: " + String(connected_devices.size()));

        delay(500);
        p_server->startAdvertising(); // アドバタイズを再開して、更に複数のセントラルとの接続を受付
        Serial.println("restart advertising..");
        remote_print("restart advertising..");
        device_connected = true;
    }

    /// @brief bluetooth通信の切断時
    /// @param p_server BLEサーバへのポインタ
    void BluetoothCommunicator::onDisconnect(BLEServer *p_server) {
        Serial.println("disconnected!");
        remote_print("disconnected!");

        // 接続中のデバイスの数を表示
        auto connected_devices = p_server->getPeerDevices(false); // この引数falseは内部で無視されている
        Serial.print("connected devices: ");
        Serial.println(String(connected_devices.size()));
        remote_print("connected devices: " + String(connected_devices.size()));

        delay(500);
        p_server->startAdvertising();
        Serial.println("start advertising..");
        remote_print("start advertising..");
        device_connected = false;
    }

    /// @brief bluetooth通信の受信時
    /// @param p_characteristic 受信したCharacteristic
    void BluetoothCommunicator::onWrite(BLECharacteristic *p_characteristic) {
        // 受信データを処理
        String rx_buf = String(p_characteristic->getValue().c_str());
        String side;
        joystick_input::JoystickInput joystick_input = parse_json_of_joystick_input(rx_buf, &side);
        if (side == "l") {
            joystick_l_input = joystick_input;
        } else if (side == "r") {
            joystick_r_input = joystick_input;
        } else {
            Serial.println("error: invalid side");
            remote_print("error: invalid side");
        }

        // ジョイスティックの入力をモニターにそのまま送信(バッファの圧迫を防ぐため、一定確率で)
        if (random(100) <= JOYSTICK_RESEND_RATE) {
            if (p_tx_characteristic == nullptr) {
                Serial.println("error: tx_characteristic is null");
                return;
            }
            JsonDocument doc;
            doc["type"] = "joystickInput";
            doc["side"] = side;
            doc["x"] = joystick_input.get_input()->x;
            doc["y"] = joystick_input.get_input()->y;
            doc["leveledX"] = joystick_input.get_leveled_input()->x;
            doc["leveledY"] = joystick_input.get_leveled_input()->y;
            doc["distance"] = joystick_input.get_distance();
            doc["angle"] = joystick_input.get_angle();
            String tx_buf;
            serializeJson(doc, tx_buf);
            p_tx_characteristic->setValue(tx_buf.c_str());
            p_tx_characteristic->notify();
        }
    }

    /// @brief モニターのコンソールにテキストを送信
    /// @param text 送信するテキスト
    void BluetoothCommunicator::remote_print(String text) {
        if (p_tx_characteristic == nullptr) {
            Serial.println("error: tx_characteristic is null");
            return;
        }

        JsonDocument doc;
        doc["type"] = "print";
        doc["text"] = text;
        String tx_json_string;
        serializeJson(doc, tx_json_string);
        p_tx_characteristic->setValue(tx_json_string.c_str());
        p_tx_characteristic->notify();
    }

    /// @brief モニターにモータのフィードバック値を送信
    void BluetoothCommunicator::remote_send_m3508_feedback(float angle, int16_t rpm, int16_t amp, uint8_t temp) {
        if (p_tx_characteristic == nullptr) {
            Serial.println("error: tx_characteristic is null");
            return;
        }

        JsonDocument doc;
        doc["type"] = "m3508Feedback";
        doc["angle"] = angle;
        doc["rpm"] = rpm;
        doc["amp"] = amp;
        doc["temp"] = temp;
        String tx_json_string;
        serializeJson(doc, tx_json_string);
        p_tx_characteristic->setValue(tx_json_string.c_str());
        p_tx_characteristic->notify();
    }

    /// @brief ジョイスティック入力のjsonをパース
    /// @param json_string bluetoothで受信したjson文字列
    /// @param side bluetoothで受信したジョイスティックの左右を代入
    /// @return ジョイスティックの入力値
    joystick_input::JoystickInput
    BluetoothCommunicator::parse_json_of_joystick_input(String json_string, String *side) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, json_string);
        if (error) {
            Serial.print("deserialization error: ");
            Serial.print(error.c_str());
        }
        *side = ((const char *)(doc["side"] | "#"))[0]; // see: https://arduinojson.org
        utils::Vec2 input = utils::Vec2(doc["x"], doc["y"]);
        utils::Vec2 leveled_input = utils::Vec2(doc["leveledX"], doc["leveledY"]);
        float distance = doc["distance"];
        float angle = doc["angle"];
        return joystick_input::JoystickInput(input, leveled_input, distance, angle);
    }

} // namespace bluetooth_communication