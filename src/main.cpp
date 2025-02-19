#include "bluetooth_communication/bluetooth_communicator.hpp"
#include "m3508_controller/m3508_controller.hpp"
#include <Arduino.h>

auto p_bluetooth_communicator = new bluetooth_communication::BluetoothCommunicator();
auto p_m3508_controller = new m3508_controller::M3508Controller(
    // モニタに情報を送信する各関数を渡す
    [](String text) { p_bluetooth_communicator->remote_print(text); },
    [](float angle, int16_t rpm, int16_t amp, uint8_t temp) {
        p_bluetooth_communicator->remote_send_m3508_feedback(angle, rpm, amp, temp);
    },
    [](float output, float p, float i, float d, float target_rpm, float error) {
        p_bluetooth_communicator->remote_send_m3508_pid_fields(output, p, i, d, target_rpm, error);
    }
);

void setup() {
    Serial.begin(115200);
    randomSeed(0); // 乱数生成器のシード値を設定
    try {
        p_bluetooth_communicator->setup();
        p_m3508_controller->setup();
    } catch (const std::exception &e) {
        Serial.print("Unhandled error in setup: ");
        Serial.println(e.what());
        p_bluetooth_communicator->remote_print("Unhandled error in setup: " + String(e.what()));
    }
}

void loop() {
    try {
        p_bluetooth_communicator->loop();
        p_m3508_controller->loop();
    } catch (const std::exception &e) {
        Serial.print("Unhandled error in loop: ");
        Serial.println(e.what());
        p_bluetooth_communicator->remote_print("Unhandled error in loop: " + String(e.what()));
    }
    delay(1);
}