#pragma once

#include <Arduino.h>
#include <bt_communication/peripheral.hpp>

namespace log {
    class Logger {
    public:
        Logger(bool enabled = true, bt_communication::BtPrinter &bt_printer);
        void println(const char *message) const;
        void println(const String message) const;
        void println(const uint64_t message) const;
        void println(const int64_t message) const;
        void println(const uint32_t message) const;
        void println(const int32_t message) const;
        void println(const uint16_t message) const;
        void println(const int16_t message) const;
        void println(const uint8_t message) const;
        void println(const int8_t message) const;
        void println(const float message) const;
        void println(const double message) const;

    private:
        bool enabled;
        bt_communication::BtPrinter &bt_printer;
    };
} // namespace log