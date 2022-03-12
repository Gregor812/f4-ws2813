#include "Random.hpp"

#ifdef STM32F429xx
    #include "Impl/RandomImplStF429.hpp"
#else
    #error "There is no implementation for Random"
#endif

namespace Peripherals
{
    uint32_t Random::GetUint32() noexcept
    {
        return RandomImpl::GetUint32();
    }

    uint32_t Random::GetUint32(uint32_t min, uint32_t max) noexcept
    {
        return RandomImpl::GetUint32(min, max);
    }
}
