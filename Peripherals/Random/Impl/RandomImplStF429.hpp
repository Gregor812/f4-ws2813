#pragma once

#include <cstdint>
#include "stm32f4xx.h"

namespace Peripherals
{
    class RandomImpl final
    {
    public:

        static uint32_t GetUint32() noexcept
        {
            return Instance().GetUint32Inst();
        }

        static uint32_t GetUint32(uint32_t min, uint32_t max) noexcept
        {
            return Instance().GetUint32Inst(min, max);
        }

        RandomImpl(const RandomImpl&) = delete;
        RandomImpl& operator=(const RandomImpl&) = delete;
        RandomImpl(const RandomImpl&&) = delete;
        RandomImpl& operator=(const RandomImpl&&) = delete;

    private:

        RandomImpl() noexcept 
        {
            RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
            RNG->CR |= RNG_CR_RNGEN;
        }

        uint32_t GetUint32Inst() noexcept
        {
            return GetValue();
        }

        uint32_t GetUint32Inst(uint32_t min, uint32_t max) noexcept
        {
            if (max < min) return 0;
            if (max == min) return max;
            return (GetValue() % (max - min)) + min;
        }
        
        static uint32_t GetValue() noexcept
        {
            while (!(RNG->SR & RNG_SR_DRDY));
            return RNG->DR;
        }

        static RandomImpl& Instance(void) noexcept
        {
            static RandomImpl instance;
            return instance;
        }
    };
}
