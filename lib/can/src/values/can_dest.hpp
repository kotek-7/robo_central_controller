#pragma once

#include "can_id.hpp"

namespace can
{
    /// @brief モジュールとCAN IDを対応させるenumです。
    enum class CanDest : can::CanId
    {
        central = 0x000,
        dc_0 = 0x100,
        dc_1 = 0x101,
        dc_2 = 0x102,
        power_0 = 0x400,
        power_1 = 0x401,
        servo_cone = 0x300,
        servo_ball = 0x301,
        m3508_1 = 0x201,
        m3508_2 = 0x202,
        m3508_3 = 0x203,
        m3508_4 = 0x204,
    };
    
    constexpr bool operator==(const can::CanId id, const can::CanDest dest)
    {
        return id == static_cast<can::CanId>(dest);
    }

    constexpr bool operator==(const can::CanDest dest, const can::CanId id)
    {
        return id == static_cast<can::CanId>(dest);
    }
} // namespace can
