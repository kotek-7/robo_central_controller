#pragma once

#include <Arduino.h>
#include <bt_communication/peripheral.hpp>

namespace log {
    class Logger {
    public:
        Logger(bool enabled = true, bt_communication::BtPrinter &bt_printer);
        void println(const char *message);
        void println(const String message);
        void println(const uint64_t message);
        void println(const int64_t message);
        void println(const uint32_t message);
        void println(const int32_t message);
        void println(const uint16_t message);
        void println(const int16_t message);
        void println(const uint8_t message);
        void println(const int8_t message);
        void println(const float message);
        void println(const double message);

    private:
        bool enabled;
        bt_communication::BtPrinter &bt_printer;
    };
} // namespace log