#pragma once

#include <Arduino.h>

namespace m3508_controller
{

    void setup();
    void loop();

    void milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8]);
    void derive_feedback_fields(const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp);
}