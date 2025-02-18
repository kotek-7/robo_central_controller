#include <Arduino.h>
#include "bluetooth_communication/bluetooth_communicator.hpp"
#include "m3508_controller/m3508_controller.hpp"

auto p_bluetooth_communicator = new bluetooth_communication::BluetoothCommunicator();
auto p_m3508_controller = new m3508_controller::M3508Controller(
    [](String text)
    {
        p_bluetooth_communicator->remote_print(text);
    },
    [](float angle, int16_t rpm, int16_t amp, uint8_t temp)
    {
        p_bluetooth_communicator->remote_send_m3508_feedback(angle, rpm, amp, temp);
    });

void setup()
{
    try
    {
        Serial.begin(115200);
        randomSeed(0); // 乱数生成器のシード値を設定
        p_bluetooth_communicator->setup();
        p_m3508_controller->setup();
    }
    catch (const std::exception &e)
    {
        Serial.print("Unhandled error in setup: ");
        Serial.println(e.what());
        p_bluetooth_communicator->remote_print(e.what());
    }
}

void loop()
{
    try
    {
        p_bluetooth_communicator->loop();
        p_m3508_controller->loop();
    }
    catch (const std::exception &e)
    {
        Serial.print("Unhandled error in loop: ");
        Serial.println(e.what());
        p_bluetooth_communicator->remote_print(e.what());
    }
}