#include "logger.hpp"

namespace log {
    Logger::Logger(bool enabled = true, bt_communication::BtPrinter &bt_printer) :
        enabled(enabled), bt_printer(bt_printer) {}

    void Logger::println(const char *message) {
        Serial.println(message);
        bt_printer.remote_print(message);
    }

    void Logger::println(const String message) {
        println(message.c_str());
    }
    void Logger::println(const uint64_t message) {
        println(String(message));
    }

    void Logger::println(const int64_t message) {
        println(String(message));
    }

    void Logger::println(const uint32_t message) {
        println(String(message));
    }
    void Logger::println(const int32_t message) {
        println(String(message));
    }
    void Logger::println(const uint16_t message) {
        println(String(message));
    }
    void Logger::println(const int16_t message) {
    }
    void Logger::println(const uint8_t message) {
        println(String(message));
    }
    void Logger::println(const int8_t message) {
        println(String(message));
    }
    void Logger::println(const float message) {
        println(String(message));
    }
    void Logger::println(const double message) {
        println(String(message));
    }
} // namespace log
