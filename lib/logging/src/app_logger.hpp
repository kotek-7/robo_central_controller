#pragma once

#include "logger.hpp"

namespace logging {
    class AppLogger: Logger {
    public:
        AppLogger(bt_communication::BtPrinter &bt_printer, bool enabled = true);
        void log(const char *message) const override;
        void log(const String message) const override;
        void log(const uint64_t message) const override;
        void log(const int64_t message) const override;
        void log(const uint32_t message) const override;
        void log(const int32_t message) const override;
        void log(const uint16_t message) const override;
        void log(const int16_t message) const override;
        void log(const uint8_t message) const override;
        void log(const int8_t message) const override;
        void log(const float message) const override;
        void log(const double message) const override;

    private:
        bool enabled;
        bt_communication::BtPrinter &bt_printer;
    };
} // namespace log