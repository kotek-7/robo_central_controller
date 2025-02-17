#include <ESP32-TWAI-CAN.hpp>
#include "pid_controller/pid_controller.hpp"

// モータの制御処理
namespace m3508_controller
{
    constexpr uint8_t CAN_TX = 16;
    constexpr uint8_t CAN_RX = 4;

    constexpr uint16_t CAN_ID = 0x200;
    constexpr uint32_t CAN_SEND_INTERVAL = 100;
    constexpr uint32_t CAN_RECEIVE_INTERVAL = 100;
    constexpr uint32_t SERIAL_READ_INTERVAL = 100;

    // PID制御用定数
    constexpr float KP = 0.5f;
    constexpr float KI = 0.0003f;
    constexpr float KD = 40;
    constexpr float CLAMPING_OUTPUT = 2000;

    /// @brief 前回のCAN送信時間
    uint32_t previous_can_send_millis;
    /// @brief 前回のCAN受信時間
    uint32_t previous_can_receive_millis;
    /// @brief 前回のシリアル受信時間
    uint32_t previous_serial_read_millis;

    void setup()
    {
        pinMode(13, OUTPUT);
        Serial.begin(115200);

        ESP32Can.setRxQueueSize(5);
        ESP32Can.setTxQueueSize(5);
        ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

        ESP32Can.setPins(CAN_TX, CAN_RX);
        if (ESP32Can.begin())
        {
            Serial.println("Init OK!");
        }
        else
        {
            Serial.println("Init Fail!");
        }
        previous_can_send_millis = millis();
        previous_can_receive_millis = millis();
    }

    void loop()
    {
        static pid_controller::PIDController pid_controller{KP, KI, KD, CLAMPING_OUTPUT, CAN_SEND_INTERVAL};

        /// @brief モータに送信する電流値(mA)
        static int32_t command_currents[4] = {0, 0, 0, 0};

        /// @brief loop()の実行回数
        static uint32_t count;

        // CANで制御量を送信
        if ((millis() - previous_can_send_millis) > CAN_SEND_INTERVAL)
        {
            command_currents[0] = pid_controller.update_output();
            uint8_t tx_buf[8];
            milli_amperes_to_bytes(command_currents, tx_buf);

            CanFrame tx_frame;
            tx_frame.identifier = CAN_ID;
            tx_frame.extd = 0;
            tx_frame.data_length_code = 8;
            tx_frame.data[0] = tx_buf[0];
            tx_frame.data[1] = tx_buf[1];
            tx_frame.data[2] = tx_buf[2];
            tx_frame.data[3] = tx_buf[3];
            tx_frame.data[4] = tx_buf[4];
            tx_frame.data[5] = tx_buf[5];
            tx_frame.data[6] = tx_buf[6];
            tx_frame.data[7] = tx_buf[7];
            ESP32Can.writeFrame(tx_frame);

            previous_can_send_millis = millis();
        }

        // CANでフィードバック値を受信
        if ((millis() - previous_can_receive_millis) > CAN_RECEIVE_INTERVAL)
        {
            CanFrame rx_frame;
            if (ESP32Can.readFrame(rx_frame, 1000))
            {
                uint32_t rx_id = rx_frame.identifier;

                uint32_t controller_id = rx_id - 0x200;
                float angle;
                int16_t rpm;
                int16_t amp;
                uint8_t temp;
                derive_feedback_fields(rx_frame.data, &angle, &rpm, &amp, &temp);

                pid_controller.set_feedback_values(angle, rpm, amp, temp);
            }
            previous_can_receive_millis = millis();
        }

        // シリアル通信で制御目標値を受信
        if ((millis() - previous_serial_read_millis) > SERIAL_READ_INTERVAL)
        {
            if (Serial.available())
            {
                delay(1); // 一連のシリアル信号をすべて受信するまで待つ
                String input_string = "";
                while (Serial.available() > 0)
                {
                    char input_char = Serial.read();
                    input_string.concat(input_char);
                }
                pid_controller.set_target_rpm(input_string.toInt());
                Serial.print("Set target rpm to: ");
                Serial.print(input_string);
                Serial.print("\n\n");
            }
            previous_serial_read_millis = millis();
        }

        digitalWrite(13, HIGH);
        count++;
    }

    /// @brief 4つの電流値を、CANで速度コントローラに送信するデータへ変換
    /// @param milli_amperes 4つの-20000\~20000の電流値(mA)を格納した配列 (要素番号と速度コントローラIDが対応)
    /// @param out_tx_buf 結果の書き込み用配列
    void milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8])
    {
        uint8_t i;
        for (i = 0; i < 4; i++)
        {
            int32_t milli_ampere = milli_amperes[i] * 16384 / 20000;
            uint8_t upper = (milli_ampere >> 8) & 0xFF;
            uint8_t lower = milli_ampere & 0xFF;
            out_tx_buf[i * 2] = upper;
            out_tx_buf[i * 2 + 1] = lower;
        }
    }

    /// @brief 速度コントローラから受け取ったデータから、フィードバック値を導出
    /// @param rx_buf CANで受信した配列
    /// @param out_angle ロータの角度(0°\~360°) (結果書き込み用)
    /// @param out_rpm 回転速度(rpm) (結果書き込み)
    /// @param out_amp 実際のトルク電流(?) (結果書き込み用)
    /// @param out_temp モータの温度(℃) (結果書き込み用)
    void derive_feedback_fields(const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp)
    {
        *out_angle = (float)(rx_buf[0] << 8 | rx_buf[1]) * 360.0f / 8191.0f;
        *out_rpm = rx_buf[2] << 8 | rx_buf[3];
        *out_amp = rx_buf[4] << 8 | rx_buf[5];
        *out_temp = rx_buf[6];
    }

}