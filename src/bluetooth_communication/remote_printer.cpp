#include <Arduino.h>
#include "./remote_printer.hpp"

namespace bluetooth_communication
{
    RemotePrinter::RemotePrinter(bluetooth_communication::BluetoothCommunicator *p_bluetooth_communicator) : p_bluetooth_communicator(p_bluetooth_communicator)
    {
    }
    void RemotePrinter::remote_print(String text)
    {
        if (p_bluetooth_communicator == nullptr)
        {
            return;
        }
        p_bluetooth_communicator->remote_print(text);
    }
}
