#include <Arduino.h>
#include "bluetooth_communication/bluetooth_communicator.hpp"
#include "bluetooth_communication/remote_printer.hpp"
#include "m3508_controller/m3508_controller.hpp"

auto p_bluetooth_communicator = new bluetooth_communication::BluetoothCommunicator();
auto p_remote_printer = new bluetooth_communication::RemotePrinter(p_bluetooth_communicator);

void setup()
{
    try {
        Serial.begin(115200);
        p_bluetooth_communicator->setup();
        m3508_controller::setup();
    } catch (const std::exception& e) {
        Serial.print("Unhandled error in setup: ");
        Serial.println(e.what());
        p_bluetooth_communicator->remote_print(e.what());
    }
}

void loop()
{
    try {
        p_bluetooth_communicator->loop();
        m3508_controller::loop();
    } catch (const std::exception& e) {
        Serial.print("Unhandled error in loop: ");
        Serial.println(e.what());
        p_bluetooth_communicator->remote_print(e.what());
    }
}