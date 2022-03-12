#include "App.hpp"
#include "Peripherals.hpp"

using namespace Peripherals;

#define OFF {34, 34, 34, 34, 34, 34, 34, 34}
#define FULL {67, 67, 67, 67, 67, 67, 67, 67}
#define HALF {34, 34, 34, 34, 67, 67, 67, 67}
#define PAUSE { \
            {0, 0, 0, 0, 0, 0, 0, 0},\
            {0, 0, 0, 0, 0, 0, 0, 0},\
            {0, 0, 0, 0, 0, 0, 0, 0},\
        },\
        {\
            {0, 0, 0, 0, 0, 0, 0, 0},\
            {0, 0, 0, 0, 0, 0, 0, 0},\
            {0, 0, 0, 0, 0, 0, 0, 0},\
        },\
        {\
            {0, 0, 0, 0, 0, 0, 0, 0},\
            {0, 0, 0, 0, 0, 0, 0, 0},\
            {0, 0, 0, 0, 0, 0, 0, 0},\
        }
#define LED_NUMBER 5
#define PAUSE_LENGTH 3
#define COLORS_NUMBER 3
#define COLOR_DEPTH 8

namespace Application
{
    static uint16_t LedColors[LED_NUMBER + PAUSE_LENGTH][COLORS_NUMBER][COLOR_DEPTH] = 
    {
        { HALF, OFF, OFF },
        { OFF, HALF, OFF },
        { OFF, OFF, HALF },
        { HALF, OFF, HALF },
        { OFF, HALF, HALF },
        PAUSE,
    };

    void App::Run()
    {
        ConfigureSystemClock();
        InitializeDevice();

        while (true)
        {
            volatile uint32_t countdown = 0xFFFFFF;
            while (--countdown);

            for (auto i = 0; i < LED_NUMBER; ++i)
            {
                for (auto j = 0; j < COLORS_NUMBER; ++j)
                {
                    for (auto k = 3; k < COLOR_DEPTH; ++k)
                    {
                        if (Random::GetUint32() % 2)
                        {
                            LedColors[i][j][k] = 67;
                        }
                        else
                        {
                            LedColors[i][j][k] = 34;
                        }
                    }
                }
            }
        }
    }

    void App::ConfigureSystemClock(void)
    {
        // enable HSE
        RCC->CR |= RCC_CR_HSEON;
        // reset PLL configuration
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk
            | RCC_PLLCFGR_PLLQ_Msk | RCC_PLLCFGR_PLLP_Msk);
        // set HSE as PLL source
        // f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = f(HSE) * 42
        // f(PLL general clock output) = f(VCO clock) / PLLP = f(HSE) * 21
        // f(USB OTG FS, SDIO, RNG clock output) = f(VCO clock) / PLLQ = f(HSE) * 6
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE | (4 << RCC_PLLCFGR_PLLM_Pos)
            | (168 << RCC_PLLCFGR_PLLN_Pos) | (7 << RCC_PLLCFGR_PLLQ_Pos);
        FLASH->ACR |= 5;
        // wait until HSE is ready
        while ((RCC->CR & RCC_CR_HSERDY) == 0);

        // enable PLL
        RCC->CR |= RCC_CR_PLLON;
        // f(APB1 or APB2) = f(PLL general clock output) / 4 = f(HSE) * 5.25
        RCC->CFGR |= (5 << RCC_CFGR_PPRE1_Pos) | (5 << RCC_CFGR_PPRE2_Pos);
        // wait until PLL is ready
        while ((RCC->CR & RCC_CR_PLLRDY) == 0);

        // set PLL as clock sourse
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        // wait until system clock is switched to PLL source
        while ((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0);
        // disable HSI
        RCC->CR &= ~(RCC_CR_HSION);
    }

    void App::InitializeDevice(void)
    {
        InitGpio();
        InitTimers();
        InitDma();
    }

    void App::InitGpio(void)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

        Gpio::Configure<Gpio::Pin::Pin8>(GPIOB, Gpio::Mode::OutputAF, Gpio::Speed::Low);
        Gpio::SetPulling<Gpio::Pin::Pin8>(GPIOB, Gpio::Pulling::PullDown);
        Gpio::SetAltFunction<Gpio::Pin::Pin8>(GPIOB, Gpio::AltFunction::AF2);
    }

    void App::InitTimers(void)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN_Msk;

        // 1.25 us cycle
        TIM4->ARR = 105 - 1;
        // output compare 3, PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT < TIMx_CCR3 else inactive
        // enable preload
        TIM4->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
        // DMA request enable
        TIM4->DIER |= TIM_DIER_CC3DE;
        // capture/compare 3 enable
        TIM4->CCER |= TIM_CCER_CC3E;
        // timer enable
        TIM4->CR1 |= TIM_CR1_CEN;
    }

    void App::InitDma(void)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

        DMA1_Stream7->PAR = (uint32_t)&(TIM4->CCR3);
        DMA1_Stream7->M0AR = (uint32_t)&LedColors;
        DMA1_Stream7->NDTR = (LED_NUMBER + PAUSE_LENGTH) * COLORS_NUMBER * COLOR_DEPTH;
        DMA1_Stream7->CR |= (2 << DMA_SxCR_CHSEL_Pos) // select channel
            | (1 << DMA_SxCR_MSIZE_Pos) // memory data size 16-bit
            | (1 << DMA_SxCR_PSIZE_Pos) // peripheral data size 16-bit
            //| DMA_SxCR_PFCTRL // peripheral flow control enable
            | DMA_SxCR_MINC // memory address pointer autoincrement enable (increment is done according to MSIZE)
            | DMA_SxCR_CIRC  // circular mode enabled
            | (1 << DMA_SxCR_DIR_Pos);  // memory-to-peripheral

        DMA1_Stream7->FCR |= DMA_SxFCR_FEIE // FIFO error interrupt enable
            | DMA_SxFCR_DMDIS // direct mode disable (enable FIFO)
            | (1 << DMA_SxFCR_FTH_Pos); // set FIFO threshold to 
        // enable stream
        DMA1_Stream7->CR |= DMA_SxCR_EN;
    }
}
