/*
 * Povide PWM based output using either Hardware or Software PWM.
 * GA 14/8/2020
 */
#include <CoreImp.h> 
#include "HybridPWM.h"
#define PWM_MAX_DUTY_CYCLE          4095
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static HardwarePWM PWMChans[MaxPWMChannels];

// Create the timers we can use
HardwareTimer Timer1(TIM1);
HardwareTimer Timer2(TIM2);
HardwareTimer Timer4(TIM4);
HardwareTimer Timer8(TIM8);
HardwareTimer Timer12(TIM12);
HardwareTimer Timer13(TIM13);
HardwareTimer Timer14(TIM14);
#if STM32H7
HardwareTimer Timer5(TIM5);
HardwareTimer Timer15(TIM15);
HardwareTimer Timer16(TIM16);
HardwareTimer Timer17(TIM17);
static HardwareTimer* PWMTimers[] = {
                                        &Timer1, &Timer2, &Timer4, &Timer5, &Timer8, &Timer12,
                                        &Timer13, &Timer14, &Timer15, &Timer16, &Timer17
};
#else
HardwareTimer Timer3(TIM3);
HardwareTimer Timer9(TIM9);
HardwareTimer Timer10(TIM10);
HardwareTimer Timer11(TIM11);
static HardwareTimer* PWMTimers[] = {
                                        &Timer2, &Timer3, &Timer4, &Timer8, &Timer12, &Timer13, &Timer14,
                                        &Timer9, &Timer10, &Timer11
};
#endif

HardwarePWM::HardwarePWM() noexcept : timer(nullptr), channel(0)
{
} 

void HardwarePWM::free() noexcept
{
    //debugPrintf("Free timer chan %d\n", channel);
    if (timer) 
    {
        timer->pause();
        timer->setMode(channel, TIMER_DISABLED);
        timer->resume();
    }
    timer = nullptr;
    channel = 0;
}

HybridPWMBase *HardwarePWM::allocate(Pin pin, uint32_t freq, float value) noexcept
{
    //debugPrintf("HWPWM allocate pin %x, freq %d\n", static_cast<int>(pin), static_cast<int>(freq));
    // search for all of the possible timers that can drive this pin
    for(const PinMap *PMEntry = PinMap_PWM; (PMEntry = pinmap_find_entry(nullptr, pin, PMEntry)) != nullptr; PMEntry++)
    {
        // Get the hardware device
        TIM_TypeDef *instance = (TIM_TypeDef *)PMEntry->peripheral;
        // Find the timer that wraps it
        HardwareTimer *timer = nullptr;
        for(uint32_t i = 0; i < ARRAY_SIZE(PWMTimers); i++)
        {
            if (PWMTimers[i]->getHandle()->Instance == instance)
            {
                timer = PWMTimers[i];
                break;
            }
        }
        if (timer == nullptr || instance == nullptr)
        {
            debugPrintf("Unable to get hardware timer for pin %x\n", static_cast<int>(pin));
            continue;
        }
        // Get the channel we need
        uint32_t chan = STM_PIN_CHANNEL(PMEntry->function);
        // Now search to see if the timer is already in use and if it is can we reuse it
        // Note that there may be multiple entries each using a different channel
        int free = -1;
        for(uint32_t i = 0; i < MaxPWMChannels; i++)
        {
            if (PWMChans[i].timer == timer)
            {
                // is the channel already in use, or is the timer the wrong frequency
                if (PWMChans[i].channel == chan || PWMChans[i].pwmPin->freq != freq)
                {
                    // we can't use this timer
                    timer = nullptr;
                    break;
                }
            }
            // make a note of a free slot we can use
            if (free < 0 && PWMChans[i].timer == nullptr)
                free = (int)i;
        }
        // do we stil have a usable timer?
        if (timer != nullptr)
        {
            // do we have a free slot (should always be the case)
            if (free < 0) return nullptr;
            //debugPrintf("Allocated slot %d timer index %d id %d chan %d to pin %x\n", free, index, get_timer_id((timer_index_t)index), chan, pin);
            // If we get here then we can use the hardware and we have a free slot
            PWMChans[free].timer = timer;
            PWMChans[free].channel = chan;
            if (freq != 0)
            {
                // set the hardware up ready to go
                timer->pause();
                timer->setMode(chan, TIMER_OUTPUT_COMPARE_PWM1, PMEntry);
                timer->setOverflow(freq, HERTZ_FORMAT);
                timer->setCaptureCompare(chan, (uint32_t)(value*PWM_MAX_DUTY_CYCLE), RESOLUTION_12B_COMPARE_FORMAT);
                timer->resume();
            }
            return &PWMChans[free];
        }
    }
    return nullptr;
}

void HardwarePWM::setValue(float value) noexcept
{
    timer->setCaptureCompare(channel, (uint32_t)(value*PWM_MAX_DUTY_CYCLE), RESOLUTION_12B_COMPARE_FORMAT);
}

void HardwarePWM::appendStatus(const StringRef& reply) noexcept
{
    reply.catf(" Tim %d chan %d", static_cast<int>(timer->getTimerId()), static_cast<int>(channel));
}
