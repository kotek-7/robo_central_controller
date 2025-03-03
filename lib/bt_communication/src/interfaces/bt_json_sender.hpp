#pragma once

#include <ArduinoJson.h>

namespace bt_communication {
    class BtJsonSender {
    public:
        virtual ~BtJsonSender() = default;

        virtual void remote_send_json(JsonDocument doc) = 0;
    };
} // namespace bt_communication
