#pragma once

#include <Arduino.h>
#include <bt_communication/peripheral.hpp>

namespace log {
    class Logger {
    public:
        Logger(bool enabled = true, bt_communication::BtPrinter &bt_printer);
        void log(const char *message) const;
        void log(const String message) const;
        void log(const uint64_t message) const;
        void log(const int64_t message) const;
        void log(const uint32_t message) const;
        void log(const int32_t message) const;
        void log(const uint16_t message) const;
        void log(const int16_t message) const;
        void log(const uint8_t message) const;
        void log(const int8_t message) const;
        void log(const float message) const;
        void log(const double message) const;

    private:
        bool enabled;
        bt_communication::BtPrinter &bt_printer;
    };
} // namespace log