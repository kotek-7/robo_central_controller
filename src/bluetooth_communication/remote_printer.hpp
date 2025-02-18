#pragma once
#include <Arduino.h>
#include "./bluetooth_communicator.hpp"

namespace bluetooth_communication
{
    class RemotePrinter
    {
    public:
        RemotePrinter(bluetooth_communication::BluetoothCommunicator *p_bluetooth_communicator);
        void remote_print(String text);
    private:
        bluetooth_communication::BluetoothCommunicator *p_bluetooth_communicator;
    };
}
