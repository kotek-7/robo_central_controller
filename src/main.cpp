#include "bluetooth_communication/bluetooth_communication.hpp"
#include "m3508_controller/m3508_controller.hpp"

void setup()
{
    Serial.begin(115200);
    bluetooth_communication::setup();
    m3508_controller::setup();
}

void loop()
{
    bluetooth_communication::loop();
    m3508_controller::loop();
}