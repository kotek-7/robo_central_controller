#include "robo_controller/robo_controller.hpp"
#include "m3508_controller/m3508_controller.hpp"

void setup()
{
    robo_controller::setup();
    m3508_controller::setup();
}

void loop()
{
    robo_controller::loop();
    m3508_controller::loop();
}