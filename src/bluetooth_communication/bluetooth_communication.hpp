#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

namespace bluetooth_communication
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

    void parse_rx_json_data(
        String jsonString,
        uint8_t *side,
        float *x,
        float *y,
        float *leveled_x,
        float *leveled_y,
        float *distance,
        float *angle);

    String create_tx_json_data(
        uint8_t side,
        float x,
        float y,
        float leveled_x,
        float leveled_y,
        float distance,
        float angle);

    void remote_print(String text);
}