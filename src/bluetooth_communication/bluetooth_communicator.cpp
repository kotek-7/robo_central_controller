
// https://qiita.com/takudooon/items/2ab77f22196504ff9597
// https://qiita.com/umi_kappa/items/dd3d7a27cf714971406e

#include "joystick_input.hpp"
#include "bluetooth_communicator.hpp"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// コントローラーとの通信処理
namespace bluetooth_communication
{
    // https://www.uuidgenerator.net/
    constexpr const char *SERVICE_UUID = "0a133f79-efe1-40c5-b4a5-cba5980d0d0f";
    constexpr const char *TX_CHARACTERISTIC_UUID = "7687561d-1dba-458f-9fb2-58e6b85208ef";
    constexpr const char *RX_CHARACTERISTIC_UUID = "8c83ffae-8421-4581-9755-10c5efd53d10";

    BluetoothCommunicator::BluetoothCommunicator() : device_connected(false), p_server(nullptr), p_tx_characteristic(nullptr), p_rx_characteristic(nullptr), joystick_l_input(joystick_input::JoystickInput()), joystick_r_input(joystick_input::JoystickInput())
    {
    }

    void BluetoothCommunicator::setup()
    {
        class ServerCallbacks : public BLEServerCallbacks
        {
        public:
            BluetoothCommunicator *p_bluetooth_communicator;

            ServerCallbacks(BluetoothCommunicator *p_bluetooth_communicator) : p_bluetooth_communicator(p_bluetooth_communicator)
            {
            }
            void onConnect(BLEServer *pServer) override
            {
                p_bluetooth_communicator->onConnect(pServer);
            }
            void onDisconnect(BLEServer *pServer) override
            {
                p_bluetooth_communicator->onDisconnect(pServer);
            }
        };

        class TxCharacteristicCallbacks : public BLECharacteristicCallbacks
        {
        };

        class RxCharacteristicCallbacks : public BLECharacteristicCallbacks
        {
        public:
            BluetoothCommunicator *p_bluetooth_communicator;

            RxCharacteristicCallbacks(BluetoothCommunicator *p_bluetooth_communicator) : p_bluetooth_communicator(p_bluetooth_communicator)
            {
            }
            void onWrite(BLECharacteristic *pCharacteristic) override
            {
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
                TX_CHARACTERISTIC_UUID,
                BLECharacteristic::PROPERTY_READ |
                    BLECharacteristic::PROPERTY_NOTIFY);
            p_tx_characteristic->setCallbacks(new TxCharacteristicCallbacks());

            // Client Characteristc Configuration Descriptor
            BLE2902 *pCccd = new BLE2902();
            p_tx_characteristic->addDescriptor(pCccd);
        }

        // 受信用Characteristicを作成
        {
            p_rx_characteristic = p_service->createCharacteristic(
                RX_CHARACTERISTIC_UUID,
                BLECharacteristic::PROPERTY_WRITE);
            p_rx_characteristic->setCallbacks(new RxCharacteristicCallbacks(this));
        }

        // 通信開始
        p_service->start();
        BLEAdvertising *p_advertising = p_server->getAdvertising();
        p_advertising->start();
    }

    void BluetoothCommunicator::loop()
    {
        // 接続中
        if (device_connected)
        {
            int random_num = random(255);
            remote_print("[sample output] random num: ");
            remote_print(String(random_num));
            Serial.println("[sample output] random num: ");
            Serial.println(String(random_num));
            delay(2000);
        }
        delay(10);
    }

    void BluetoothCommunicator::onConnect(BLEServer *p_server)
    {
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

    void BluetoothCommunicator::onDisconnect(BLEServer *p_server)
    {
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

    void BluetoothCommunicator::onWrite(BLECharacteristic *p_characteristic)
    {
        // 受信データを処理
        String rx_buf = String(p_characteristic->getValue().c_str());
        String side;
        joystick_input::JoystickInput joystick_input = parse_json_of_joystick_input(rx_buf, &side);
        if (side == "l")
        {
            joystick_l_input = joystick_input;
        }
        else if (side == "r")
        {
            joystick_r_input = joystick_input;
        }
        else
        {
            Serial.println("error: invalid side");
            remote_print("error: invalid side");
        }

        // ジョイスティックの入力をそのままモニターに送信
        if (p_tx_characteristic == nullptr)
        {
            return;
        }
        p_tx_characteristic->setValue(rx_buf.c_str());
        p_tx_characteristic->notify();
    }

    void BluetoothCommunicator::remote_print(String text)
    {
        if (p_tx_characteristic == nullptr)
        {
            return;
        }

        JsonDocument doc;
        doc["nextLine"] = text;
        String tx_json_string;
        serializeJson(doc, tx_json_string);
        p_tx_characteristic->setValue(tx_json_string.c_str());
        p_tx_characteristic->notify();
    }

    joystick_input::JoystickInput BluetoothCommunicator::parse_json_of_joystick_input(String json_string, String *side)
    {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, json_string);
        if (error)
        {
            Serial.print("deserialization error: ");
            Serial.print(error.c_str());
        }
        *side = ((const char *)(doc["side"] | "#"))[0]; // see: https://arduinojson.org
        utils::Vec2 input = utils::Vec2(doc["x"], doc["y"]);
        utils::Vec2 leveled_input = utils::Vec2(doc["leveledX"], doc["leveledY"]);
        float distance = doc["distance"];
        float angle = doc["angle"];
        return joystick_input::JoystickInput(
            input,
            leveled_input,
            distance,
            angle);
    }

    String BluetoothCommunicator::create_json_of_joystick_input(
        uint8_t side,
        float x,
        float y,
        float leveled_x,
        float leveled_y,
        float distance,
        float angle)
    {
        JsonDocument doc;
        doc["side"] = side;
        doc["x"] = x;
        doc["y"] = y;
        doc["leveledX"] = leveled_x;
        doc["leveledY"] = leveled_y;
        doc["distance"] = distance;
        doc["angle"] = angle;
        String json_string;
        serializeJson(doc, json_string);
        return json_string;
    }

}