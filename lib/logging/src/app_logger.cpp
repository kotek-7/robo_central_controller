#include "app_logger.hpp"

namespace logging {
    AppLogger::AppLogger(bt_communication::BtPrinter &bt_printer, bool enabled = true) :
        enabled(enabled), bt_printer(bt_printer) {}

    void AppLogger::log(const char *message) const {
        Serial.println(message);
        bt_printer.remote_print(message);
    }

    void AppLogger::log(const String message) const {
        log(message.c_str());
    }
    void AppLogger::log(const uint64_t message) const {
        log(String(message));
    }

    void AppLogger::log(const int64_t message) const {
        log(String(message));
    }

    void AppLogger::log(const uint32_t message) const {
        log(String(message));
    }
    void AppLogger::log(const int32_t message) const {
        log(String(message));
    }
    void AppLogger::log(const uint16_t message) const {
        log(String(message));
    }
    void AppLogger::log(const int16_t message) const {
        log(String(message));
    }
    void AppLogger::log(const uint8_t message) const {
        log(String(message));
    }
    void AppLogger::log(const int8_t message) const {
        log(String(message));
    }
    void AppLogger::log(const float message) const {
        log(String(message));
    }
    void AppLogger::log(const double message) const {
        log(String(message));
    }
} // namespace log
