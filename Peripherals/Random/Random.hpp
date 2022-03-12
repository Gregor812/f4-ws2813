#pragma once

#include <cstdint>

namespace Peripherals
{
    class Random final
    {
    public:

        static uint32_t GetUint32() noexcept;
        static uint32_t GetUint32(uint32_t min, uint32_t max) noexcept;

        Random() = delete;
        Random(const Random&) = delete;
        Random& operator=(const Random&) = delete;
        Random(const Random&&) = delete;
        Random& operator=(const Random&&) = delete;
    };
}
