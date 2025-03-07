#include "logger.hpp"

namespace log {
    Logger::Logger(bool enabled = true, bt_communication::BtPrinter &bt_printer) :
        enabled(enabled), bt_printer(bt_printer) {}

    void Logger::log(const char *message) const {
        Serial.println(message);
        bt_printer.remote_print(message);
    }

    void Logger::log(const String message) const {
        log(message.c_str());
    }
    void Logger::log(const uint64_t message) const {
        log(String(message));
    }

    void Logger::log(const int64_t message) const {
        log(String(message));
    }

    void Logger::log(const uint32_t message) const {
        log(String(message));
    }
    void Logger::log(const int32_t message) const {
        log(String(message));
    }
    void Logger::log(const uint16_t message) const {
        log(String(message));
    }
    void Logger::log(const int16_t message) const {
        log(String(message));
    }
    void Logger::log(const uint8_t message) const {
        log(String(message));
    }
    void Logger::log(const int8_t message) const {
        log(String(message));
    }
    void Logger::log(const float message) const {
        log(String(message));
    }
    void Logger::log(const double message) const {
        log(String(message));
    }
} // namespace log
