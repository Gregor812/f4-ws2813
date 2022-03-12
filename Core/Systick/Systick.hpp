#pragma once

#include <cstdint>

namespace Core
{
    class Systick final
    {
    public:

        static void DelayMilliseconds(uint32_t milliseconds) noexcept;

        Systick() = delete;
        Systick(const Systick&) = delete;
        Systick& operator=(const Systick&) = delete;
        Systick(const Systick&&) = delete;
        Systick& operator=(const Systick&&) = delete;
    };
}
