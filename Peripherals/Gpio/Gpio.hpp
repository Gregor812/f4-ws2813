#pragma once

#include <cstdint>

#include "stm32f4xx.h"

namespace Peripherals
{
    class Gpio
    {
    public:

        enum class Mode : uint8_t
        { Input, Output, OutputAF, Analog };

        enum class OutputType : uint8_t
        { PushPull, OpenDrain };

        enum class Speed : uint8_t
        { Low, Medium, High, VeryHigh };

        enum class Pulling : uint8_t
        { No, PullUp, PullDown };

        enum class AltFunction : uint8_t
        { 
            AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7,
            AF8, AF9, AF10, AF11, AF12, AF13, AF14, AF15
        };

        enum class Pin : uint8_t
        { 
            Pin0, Pin1, Pin2, Pin3, Pin4, Pin5, Pin6, Pin7,
            Pin8, Pin9, Pin10, Pin11, Pin12, Pin13, Pin14, Pin15
        };

        template<Pin... pins>
        static inline void SetMode(GPIO_TypeDef *port, Mode mode)
        {
            uint32_t temp = 0;
            ((temp |= (0b11 << (2 * static_cast<uint8_t>(pins)))), ...);
            port->MODER &= ~(temp);
            temp = 0;
            ((temp |= static_cast<uint8_t>(mode) << (2 * static_cast<uint8_t>(pins))), ...);
            port->MODER |= temp;
        }

        template<Pin... pins>
        static inline void SetOutputType(GPIO_TypeDef *port, OutputType outputType)
        {
            uint32_t temp = 0;
            ((temp |= (0b1 << (static_cast<uint8_t>(pins)))), ...);
            port->OTYPER &= ~(temp);
            temp = 0;
            ((temp |= (static_cast<uint8_t>(outputType) << (static_cast<uint8_t>(pins)))), ...);
            port->OTYPER |= temp;
        }

        template<Pin... pins>
        static inline void SetSpeed(GPIO_TypeDef *port, Speed speed)
        {
            uint32_t temp = 0;
            ((temp |= (0b11 << (2 * static_cast<uint8_t>(pins)))), ...);
            port->OSPEEDR &= ~(temp);
            temp = 0;
            ((temp |= (static_cast<uint8_t>(speed) << (2 * static_cast<uint8_t>(pins)))), ...);
            port->OSPEEDR |= temp;
        }

        template<Pin... pins>
        static inline void SetPulling(GPIO_TypeDef *port, Pulling pulling)
        {
            uint32_t temp = 0;
            ((temp |= (0b11 << (2 * static_cast<uint8_t>(pins)))), ...);
            port->PUPDR &= ~(temp);
            temp = 0;
            ((temp |= (static_cast<uint8_t>(pulling) << (2 * static_cast<uint8_t>(pins)))), ...);
            port->PUPDR |= temp;
        }

        template<Pin... pins>
        static inline void SetAltFunction(GPIO_TypeDef *port, AltFunction altFunction)
        {
            uint32_t temp1 = 0, temp2 = 0;
            (((pins < Pin::Pin8 ? temp1 : temp2) |=
                (0b11 << (4 * (static_cast<uint8_t>(pins) - (pins < Pin::Pin8 ? 0 : 8))))), ...);
            port->AFR[0] &= ~temp1;
            port->AFR[1] &= ~temp2;

            temp1 = 0, temp2 = 0;
            (((pins < Pin::Pin8 ? temp1 : temp2) |=
                (static_cast<uint8_t>(altFunction) << (4 * (static_cast<uint8_t>(pins) - (pins < Pin::Pin8 ? 0 : 8))))), ...);
            port->AFR[0] |= temp1;
            port->AFR[1] |= temp2;
        }

        template<Pin... pins>
        static inline void Configure(GPIO_TypeDef *port, Mode mode, Speed speed)
        {
            SetMode<pins...>(port, mode);
            SetSpeed<pins...>(port, speed);
        }

        template<Pin... pins>
        static inline void Configure(GPIO_TypeDef *port, Mode mode, Speed speed, AltFunction altFunction)
        {
            SetMode<pins...>(port, mode);
            SetSpeed<pins...>(port, speed);
            SetAltFunction<pins...>(port, altFunction);
        }

        template<Pin... pins>
        static inline void Set(GPIO_TypeDef *port)
        {
            uint32_t temp = 0;
            ((temp |= 1 << static_cast<uint8_t>(pins)), ...);
            port->BSRR = temp;
        }

        template<Pin... pins>
        static inline void Toggle(GPIO_TypeDef *port)
        {
            uint32_t temp = 0;
            ((temp |= 1 << static_cast<uint8_t>(pins)), ...);
            port->ODR ^= temp;
        }

        template<Pin... pins>
        static inline void Reset(GPIO_TypeDef *port)
        {
            uint32_t temp = 0;
            ((temp |= 1 << (static_cast<uint8_t>(pins) + 16)), ...);
            port->BSRR = temp;
        }
    };
}
