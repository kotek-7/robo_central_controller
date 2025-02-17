#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

namespace robo_controller
{
    class ServerCallbacks : public BLEServerCallbacks
    {
    public:
        void onConnect(BLEServer *pServer);
        void onDisconnect(BLEServer *pServer);
    };

    class TxCharacteristicCallbacks : public BLECharacteristicCallbacks
    {
    };

    class RxCharacteristicCallbacks : public BLECharacteristicCallbacks
    {
    public:
        void onWrite(BLECharacteristic *pCharacteristic);
    };

    void setup();
    void loop();

    void parseRxJsonData(
        String jsonString,
        uint8_t *side,
        float *x,
        float *y,
        float *leveled_x,
        float *leveled_y,
        float *distance,
        float *angle);

    String createTxJsonData(
        uint8_t side,
        float x,
        float y,
        float leveled_x,
        float leveled_y,
        float distance,
        float angle);

    void remotePrint(String text);
}