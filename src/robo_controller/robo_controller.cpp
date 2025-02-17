
// https://qiita.com/takudooon/items/2ab77f22196504ff9597
// https://qiita.com/umi_kappa/items/dd3d7a27cf714971406e

#include "robo_controller.hpp"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// コントローラーとの通信処理
namespace robo_controller
{
    // https://www.uuidgenerator.net/
    constexpr char *SERVICE_UUID = "0a133f79-efe1-40c5-b4a5-cba5980d0d0f";
    constexpr char *TX_CHARACTERISTIC_UUID = "7687561d-1dba-458f-9fb2-58e6b85208ef";
    constexpr char *RX_CHARACTERISTIC_UUID = "8c83ffae-8421-4581-9755-10c5efd53d10";

    bool deviceConnected = false;
    BLEServer *pServer;
    BLECharacteristic *pTxCharacteristic;
    BLECharacteristic *pRxCharacteristic;

    void ServerCallbacks::onConnect(BLEServer *pServer)
    {
        Serial.println("connected!");
        remotePrint("conncted!");
        delay(500);
        pServer->startAdvertising(); // アドバタイズを再開して、更に複数のセントラルとの接続を受付
        Serial.println("restart advertising..");
        remotePrint("restart advertising..");
        deviceConnected = true;
    }
    void ServerCallbacks::onDisconnect(BLEServer *pServer)
    {
        Serial.println("disconnected!");
        remotePrint("disconnected!");
        delay(500);
        pServer->startAdvertising();
        Serial.println("start advertising..");
        remotePrint("start advertising..");
        deviceConnected = false;
    }

    void RxCharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic)
    {
        // 受信データを処理
        String rx_buf = String(pCharacteristic->getValue().c_str());
        uint8_t side;
        float x, y, leveled_x, leveled_y, distance, angle;
        parseRxJsonData(rx_buf, &side, &x, &y, &leveled_x, &leveled_y, &distance, &angle);

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
        String txJsonString = createTxJsonData(side, x, y, leveled_x, leveled_y, distance, angle);
        pTxCharacteristic->setValue(txJsonString.c_str());
        pTxCharacteristic->notify();
    }

    void setup()
    {
        Serial.begin(115200);

        BLEDevice::init("esp32_for_BLE");
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new ServerCallbacks());

        // Serviceを作成
        BLEService *pService = pServer->createService(SERVICE_UUID);

        // 送信用Characteristicを作成
        {
            pTxCharacteristic = pService->createCharacteristic(
                TX_CHARACTERISTIC_UUID,
                BLECharacteristic::PROPERTY_READ |
                    BLECharacteristic::PROPERTY_NOTIFY);
            pTxCharacteristic->setCallbacks(new TxCharacteristicCallbacks());

            // Client Characteristc Configuration Descriptor
            BLE2902 *pCccd = new BLE2902();
            pTxCharacteristic->addDescriptor(pCccd);
        }

        // 受信用Characteristicを作成
        {
            pRxCharacteristic = pService->createCharacteristic(
                RX_CHARACTERISTIC_UUID,
                BLECharacteristic::PROPERTY_WRITE);
            pRxCharacteristic->setCallbacks(new RxCharacteristicCallbacks());
        }

        // 通信開始
        pService->start();
        BLEAdvertising *pAdvertising = pServer->getAdvertising();
        pAdvertising->start();
    }

    void loop()
    {
        // 接続中
        if (deviceConnected)
        {
            int random_num = random(255);
            remotePrint("[sample output] random num: ");
            remotePrint(String(random_num));
            Serial.println("[sample output] random num: ");
            Serial.println(String(random_num));
            delay(2000);
        }
        delay(10);
    }

    void parseRxJsonData(
        String jsonString,
        uint8_t *side,
        float *x,
        float *y,
        float *leveled_x,
        float *leveled_y,
        float *distance,
        float *angle)
    {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, jsonString);
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

    String createTxJsonData(
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
        String jsonString;
        serializeJson(doc, jsonString);
        return jsonString;
    }

    void remotePrint(String text)
    {
        JsonDocument doc;
        doc["nextLine"] = text;
        String txJsonString;
        serializeJson(doc, txJsonString);
        pTxCharacteristic->setValue(txJsonString.c_str());
        pTxCharacteristic->notify();
    }
}