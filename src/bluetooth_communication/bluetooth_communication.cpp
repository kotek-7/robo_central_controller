
// https://qiita.com/takudooon/items/2ab77f22196504ff9597
// https://qiita.com/umi_kappa/items/dd3d7a27cf714971406e

#include "bluetooth_communication.hpp"
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
    constexpr char *SERVICE_UUID = "0a133f79-efe1-40c5-b4a5-cba5980d0d0f";
    constexpr char *TX_CHARACTERISTIC_UUID = "7687561d-1dba-458f-9fb2-58e6b85208ef";
    constexpr char *RX_CHARACTERISTIC_UUID = "8c83ffae-8421-4581-9755-10c5efd53d10";

    bool device_connected = false;
    BLEServer *p_server;
    BLECharacteristic *p_tx_characteristic;
    BLECharacteristic *p_rx_characteristic;

    void ServerCallbacks::onConnect(BLEServer *pServer)
    {
        Serial.println("connected!");
        remote_print("conncted!");
        delay(500);
        pServer->startAdvertising(); // アドバタイズを再開して、更に複数のセントラルとの接続を受付
        Serial.println("restart advertising..");
        remote_print("restart advertising..");
        device_connected = true;
    }
    void ServerCallbacks::onDisconnect(BLEServer *pServer)
    {
        Serial.println("disconnected!");
        remote_print("disconnected!");
        delay(500);
        pServer->startAdvertising();
        Serial.println("start advertising..");
        remote_print("start advertising..");
        device_connected = false;
    }

    void RxCharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic)
    {
        // 受信データを処理
        String rx_buf = String(pCharacteristic->getValue().c_str());
        uint8_t side;
        float x, y, leveled_x, leveled_y, distance, angle;
        parse_rx_json_data(rx_buf, &side, &x, &y, &leveled_x, &leveled_y, &distance, &angle);

        Serial.println("******");
        Serial.print("rx buf: ");
        Serial.println(rx_buf);
        Serial.print("side: ");
        Serial.print((char)side);
        Serial.print(", x: ");
        Serial.print(x);
        Serial.print(", y: ");
        Serial.print(y);
        Serial.print(", leveled_x: ");
        Serial.print(leveled_x);
        Serial.print(", leveled_y: ");
        Serial.print(leveled_y);
        Serial.print(", distance: ");
        Serial.print(distance);
        Serial.print(", angle: ");
        Serial.println(angle);
        Serial.print("len: ");
        Serial.println(rx_buf.length());
        Serial.println("******");

        // 受信データと同じデータをつくって送信(仮の処理)
        String txJsonString = create_tx_json_data(side, x, y, leveled_x, leveled_y, distance, angle);
        p_tx_characteristic->setValue(txJsonString.c_str());
        p_tx_characteristic->notify();
    }

    void setup()
    {
        BLEDevice::init("esp32_for_BLE");
        p_server = BLEDevice::createServer();
        p_server->setCallbacks(new ServerCallbacks());

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
            p_rx_characteristic->setCallbacks(new RxCharacteristicCallbacks());
        }

        // 通信開始
        p_service->start();
        BLEAdvertising *p_advertising = p_server->getAdvertising();
        p_advertising->start();
    }

    void loop()
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

    void parse_rx_json_data(
        String json_string,
        uint8_t *side,
        float *x,
        float *y,
        float *leveled_x,
        float *leveled_y,
        float *distance,
        float *angle)
    {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, json_string);
        if (error)
        {
            Serial.print("deserialization error: ");
            Serial.print(error.c_str());
            return;
        }
        *side = ((const char *)(doc["side"] | "#"))[0]; // see: https://arduinojson.org
        *x = doc["x"];
        *y = doc["y"];
        *leveled_x = doc["leveledX"];
        *leveled_y = doc["leveledY"];
        *distance = doc["distance"];
        *angle = doc["angle"];
    }

    String create_tx_json_data(
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
        String tx_json_string;
        serializeJson(doc, tx_json_string);
        return tx_json_string;
    }

    void remote_print(String text)
    {
        JsonDocument doc;
        doc["nextLine"] = text;
        String tx_json_string;
        serializeJson(doc, tx_json_string);
        p_tx_characteristic->setValue(tx_json_string.c_str());
        p_tx_characteristic->notify();
    }
}