#pragma once

#include <Arduino.h>

namespace m3508_control {
    enum class C620Id : uint8_t { C1 = 1, C2 = 2, C3 = 3, C4 = 4 };

    class C620IdHash {
    public:
        std::size_t operator()(C620Id c620_id) const { return std::hash<int>()(static_cast<int>(c620_id)); }
    };
} // namespace m3508_control