#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "joystick_input.hpp"

namespace bluetooth_communication
{
    class BluetoothCommunicator
    {
    public:
        BluetoothCommunicator();
        void setup();
        void loop();
        joystick_input::JoystickInput get_joystick_l_input() const { return joystick_l_input; }
        joystick_input::JoystickInput get_joystick_r_input() const { return joystick_r_input; }
        void remote_print(String text);

    private:
        bool device_connected;
        BLEServer *p_server;
        BLECharacteristic *p_tx_characteristic;
        BLECharacteristic *p_rx_characteristic;
        joystick_input::JoystickInput joystick_l_input;
        joystick_input::JoystickInput joystick_r_input;

        void onConnect(BLEServer *pServer);
        void onDisconnect(BLEServer *pServer);
        void onWrite(BLECharacteristic *pCharacteristic);
        joystick_input::JoystickInput parse_json_of_joystick_input(String json_string, String *side);
        String create_json_of_joystick_input(
            uint8_t side,
            float x,
            float y,
            float leveled_x,
            float leveled_y,
            float distance,
            float angle);
    };
}