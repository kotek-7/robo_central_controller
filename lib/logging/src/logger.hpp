#pragma once

#include <Arduino.h>
#include <bt_communication/peripheral.hpp>

namespace logging {
    class Logger {
    public:
        virtual ~Logger() = default;
        virtual void log(const char *message) const = 0;
        virtual void log(const String message) const = 0;
        virtual void log(const uint64_t message) const = 0;
        virtual void log(const int64_t message) const = 0;
        virtual void log(const uint32_t message) const = 0;
        virtual void log(const int32_t message) const = 0;
        virtual void log(const uint16_t message) const = 0;
        virtual void log(const int16_t message) const = 0;
        virtual void log(const uint8_t message) const = 0;
        virtual void log(const int8_t message) const = 0;
        virtual void log(const float message) const = 0;
        virtual void log(const double message) const = 0;
    };
} // namespace log