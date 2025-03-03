#pragma once

#include <Arduino.h>

namespace bt_communication {
    class BtPrinter {
    public:
        virtual ~BtPrinter() = default;

        virtual void remote_print(String text) const = 0;
    };
}