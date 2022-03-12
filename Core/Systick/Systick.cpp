#include "Systick.hpp"

#ifdef STM32F429xx
    #include "Impl/SystickImplStF429.hpp"
#else
    #error "There is no implementation for Systick"
#endif

namespace Core
{
    void Systick::DelayMilliseconds(uint32_t milliseconds) noexcept
    {
        SystickImpl::DelayMilliseconds(milliseconds);
    }
}
