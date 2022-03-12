#pragma once

#include <cstdint>
#include "stm32f4xx.h"

namespace Core
{
    class SystickImpl final
    {
    public:
        static void DelayMilliseconds(uint32_t milliseconds) noexcept
        {
            Instance().DelayMillisecondsInst(milliseconds);
        }

        SystickImpl(const SystickImpl&) = delete;
        SystickImpl& operator=(const SystickImpl&) = delete;
        SystickImpl(const SystickImpl&&) = delete;
        SystickImpl& operator=(const SystickImpl&&) = delete;

    private:
        friend void SysTickHandler() noexcept;
        SystickImpl() noexcept
        {
            _sysTickCounter = 0;
            SysTick_Config(168000000L/1000L);
            SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
            NVIC_SetPriority(SysTick_IRQn, 0);
        }

        void DelayMillisecondsInst(uint32_t milliseconds) noexcept
        {
            uint32_t counterSnapshot = this->_sysTickCounter;
            while (_sysTickCounter - counterSnapshot < milliseconds);
        }

        static SystickImpl& Instance(void) noexcept
        {
            static SystickImpl instance;
            return instance;
        }

        volatile uint32_t _sysTickCounter;
    };

    
    inline void SysTickHandler() noexcept
    {
        SystickImpl::Instance()._sysTickCounter++;
    }
}

extern "C" inline void SysTick_Handler() noexcept
{
    Core::SysTickHandler();
}
