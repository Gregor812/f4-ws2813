#pragma once

#include <cstdint>
#include "stm32f4xx.h"

namespace Application
{
    class App final
    {
    public:
        void Run();
    private:
        void ConfigureSystemClock(void);
        void InitializeDevice(void);
        void InitGpio(void);
        void InitTimers(void);
        void InitDma(void);
    };
}
