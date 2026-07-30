/*
 * Software PWM
 * 
 * Provide PWM outputs for up to 16 channels using any gpio pin.
 * This module uses a single basic hardware timer runing at 1MHz to provide the base clock.
 * It uses an interrupt generated when the timer hits the update threshold to generate the
 * next falling/rising edge of the PWM signal as required. Channels which require a full on/off
 * value simply set the gpio as needed and the channel is marked as unused. 
 * 
 * Care must be taken to ensure that the reload value of the timer is not ever set to 0 as this
 * in effect will freeze the timer when the counter overflows back to 0. This in effect means that
 * minimum period between two updates is 2uS (as setting the reload value to n generates a reload
 * after n+1 ticks). This along with delays when other parts pf RRF diable interrupts or when a
 * higher priority interrupt is generated means that events may be a few ticks late (a typical maximum
 * is 3uS). We have two options for how to deal with this:
 * a) ignore it and set things up so that the next edge is "on time" in effect preserving the update freq.
 * b) take the lateness in to account and set things so that the next edge is also late, prioritising the pulse length.
 * This implementation chooses option b as this produces less chatter on common servos.
 * 
 * Author: Andy
 * */

#include <CoreImp.h>
#include "HybridPWM.h"
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
// NOTE: The debug error calculations assume a Step Timer running at 1MHz
#define PWM_DEBUG
# if STM32H7
# define SYNC_GPIO() __DSB()
#else
# define SYNC_GPIO()
# endif
#define SPWM_TIMER TIM7
HardwareTimer SPWMTimer(SPWM_TIMER);
// Minimum period between interrupts - in microseconds (to prevent starving other tasks)
static constexpr uint32_t MinimumInterruptDeltaUS = 0;

typedef struct {
    uint32_t nextEvent;
    Pin pin;
    uint32_t onOffTimes[2][2];
    uint8_t state;
    uint8_t onOffBuffer;
    bool newTimes;
    bool enabled;
} PWMState;

static PWMState States[MaxPWMChannels];
static int32_t startActive = -1;
static int32_t endActive = -1;
static uint32_t baseTime = 0;
static uint32_t baseDelta = 1;
static bool timerReady = false;
static TIM_HandleTypeDef *timerHandle;

static SoftwarePWM PWMChans[MaxPWMChannels];

#ifdef PWM_DEBUG
HardwareTimer dbtimer(TIM5);
uint32_t pwmInts = 0;
uint32_t pwmCalls = 0;
uint32_t pwmMinTime = 0xffffffff;
uint32_t pwmMaxTime = 0;
uint32_t pwmAdjust = 0;
uint32_t pwmBigDelta = 0;
uint32_t pwmVBigDelta = 0;
uint32_t pwmBadRange = 0;
int32_t pwmAccErr = 0;
int pwmBadVal = 0;
uint32_t pwmPending = 0;
uint32_t pwmNegADelta = 0;
uint32_t pwmNegCDelta = 0;
uint32_t pwmEarly = 0;
uint32_t pwmOneDelta = 0;
int32_t pwmMaxErr = -0x7fffffff;
int32_t pwmMinErr = 0x7fffffff;
uint32_t pwmStartTicks = 0;
uint32_t pwmBaseTicks = 0;
uint32_t pwmOOB = 0;
#endif

static void updateActive(uint32_t newChan = 0xffffffff)
{
    int32_t first = -1;
    int32_t last = -1;
    for(uint32_t i = 0; i < MaxPWMChannels; i++)
        if (States[i].enabled || i == newChan)
        {
            last = i + 1;
            if (first < 0) first = i;
        }
    endActive = last;
    startActive = first;
}

static void disable(int chan)
{
    States[chan].enabled = false;
#ifdef PWM_DEBUG
    debugPrintf("disable %d\n", chan);
#endif
    updateActive();
    if (endActive < 0)
        __HAL_TIM_DISABLE_IT(timerHandle, TIM_IT_UPDATE);
}

static int enable(Pin pin, uint32_t onTime, uint32_t offTime)
{
    // find a free slot
    uint32_t newSlot = 0;
    while(newSlot < MaxPWMChannels && States[newSlot].enabled)
    {
        newSlot++;
    }
    if (newSlot >= MaxPWMChannels)
    {
#ifdef PWM_DEBUG
        debugPrintf("PWM no free slot\n");
#endif
        return -1;
    }
#ifdef PWM_DEBUG
    debugPrintf("enable slot %d\n", (int)newSlot);
#endif
    // set up the new slot
    PWMState& s = States[newSlot];
    s.pin = pin;
    s.onOffTimes[0][0] = onTime;
    s.onOffTimes[0][1] = offTime;
    s.onOffBuffer = 0;
    s.state = 1;
    s.newTimes = false;
    updateActive(newSlot);
    int32_t syncSlot = -1;
#if 0
    uint32_t cycleTime = onTime + offTime;
    for(int32_t i = startActive; i < endActive; i++)
    {
        if (States[i].enabled && (States[i].onOffTimes[States[i].onOffBuffer][0] + States[i].onOffTimes[States[i].onOffBuffer][1]) == cycleTime)
        {
            syncSlot = i;
            break;
        }
    }
#endif
#ifdef PWM_DEBUG
    debugPrintf("found sync slot %d\n", (int)syncSlot);
#endif

    // Pause things while we complete the setup and then trigger an updated asap
    __HAL_TIM_DISABLE_IT(timerHandle, TIM_IT_UPDATE);
    __HAL_TIM_SET_AUTORELOAD(timerHandle, 0xffff);
    uint32_t curDelta = __HAL_TIM_GET_COUNTER(timerHandle);
    s.enabled = true;
    if (__HAL_TIM_GET_ITSTATUS(timerHandle, TIM_IT_UPDATE))
        // the timer has completed the planned cycle, plus whatever ticks may have happened since we diabled ints
        baseDelta += __HAL_TIM_GET_COUNTER(timerHandle);
    else
        baseDelta = curDelta;
    if (syncSlot >= 0)
    {
        PWMState& sync = States[syncSlot];
        // we have something to sync with, set our event to match
        s.nextEvent = sync.nextEvent + (sync.state == 1 ? 0 : sync.onOffTimes[sync.onOffBuffer][1]);
    }
    else
        s.nextEvent = baseTime + baseDelta;
#ifdef PWM_DEBUG
    pwmBaseTicks = dbtimer.getCount() - (baseTime + baseDelta);
#endif
    timerHandle->Instance->EGR = TIM_EVENTSOURCE_UPDATE;
    __HAL_TIM_ENABLE_IT(timerHandle, TIM_IT_UPDATE);
    return newSlot;
}

static void adjustOnOffTime(int chan, uint32_t onTime, uint32_t offTime)
{
    PWMState& s = States[chan];
#ifdef PWM_DEBUG
    if (s.newTimes) 
    {
        pwmPending++;
        uint32_t count = __HAL_TIM_GET_COUNTER(timerHandle);
        uint32_t reload = __HAL_TIM_GET_AUTORELOAD(timerHandle);
        uint32_t now = baseTime + count;
        debugPrintf("time %u/%u pending chan %d next %u now %u count %u reload %u delta %u\n", (unsigned)onTime, (unsigned)offTime, (unsigned)chan, (unsigned)States[chan].nextEvent, (unsigned)now, (unsigned)count, (unsigned)reload, (unsigned)baseDelta);
    }
#endif
    uint32_t buffer = s.onOffBuffer ^ 1;
    s.onOffTimes[buffer][0] = onTime;
    s.onOffTimes[buffer][1] = offTime;
    s.newTimes = true;
}

// We handle interrupts directly to reduce overhead so this function
// is just a placeholder
extern "C" void SPWM_Handler(HardwareTimer *) __attribute__ ((hot));
void SPWM_Handler(HardwareTimer * notused)
{
}

extern "C" void TIM7_IRQHandler(void) noexcept __attribute__((optimize("O2")));
void TIM7_IRQHandler(void) noexcept
{
    // ensure we do not reset the counter a 2nd time
    __HAL_TIM_SET_AUTORELOAD(timerHandle, 0xffff);
    uint32_t computedNow = baseTime + baseDelta;
    uint32_t actualNow = computedNow + __HAL_TIM_GET_COUNTER(timerHandle);
#ifdef PWM_DEBUG
    pwmInts++;
    const uint32_t startTime = dbtimer.getCount();
    int32_t err = (actualNow - baseTime) - (startTime - pwmStartTicks);
    pwmAccErr = actualNow - (startTime - pwmBaseTicks);
    pwmStartTicks = startTime;
    if (err > pwmMaxErr) pwmMaxErr = err;
    if (err < pwmMinErr) pwmMinErr = err;
    if (err > 5 || err < -5) pwmOOB++;
#endif
    uint32_t next = 0x7fffffff;
    baseTime = computedNow;
    for(int i = startActive; i < endActive; i++)
    {
#ifdef PWM_DEBUG
        if (i < 0 || i >= (int)MaxPWMChannels)
        {
            pwmBadRange++;
            pwmBadVal = i;
            break;
        }
#endif
        if (States[i].enabled)
        {
            PWMState& s = States[i];
            int32_t actualDelta = (s.nextEvent - actualNow);
            int32_t computedDelta = (s.nextEvent - computedNow);
//            if (actualDelta <= 0)
            if (computedDelta <= 0)
            {
                // time has expired, move to next state
                s.state ^= 1;
                const uint32_t newState = s.state;
                // do we need to switch to a new set of timing parameters?
                if (newState == 0)
                {
                    fastDigitalWriteHigh(s.pin);
                    if (s.newTimes)
                    {
                        s.onOffBuffer ^= 1;
                        s.newTimes = false;
                    }
                }
                else
                    fastDigitalWriteLow(s.pin);
                SYNC_GPIO();
#ifdef PWM_DEBUG
                pwmCalls++;
                if (actualDelta < 0)
                    pwmNegADelta++;
                if (computedDelta < 0)
                    pwmNegCDelta++;
                if (computedDelta > 0)
                    pwmEarly++;
#endif
                // adjust next time by any drift to keep things in sync
                computedDelta = s.onOffTimes[s.onOffBuffer][newState] + __HAL_TIM_GET_COUNTER(timerHandle);
                s.nextEvent += computedDelta;
                // don't allow correction to go too far!
            }
            // track the smallest delta as the next target time increment
            if ((uint32_t)computedDelta < next)
                next = (uint32_t)computedDelta;
        }
    }
    // Set the new compare value
    if (next > 0xffff)
    {
        next = 0xffff;
#ifdef PWM_DEBUG
        pwmVBigDelta++;
#endif
    }
    else if (next > 1)
        next--;
    else
    {
        next = 1;
#ifdef PWM_DEBUG
        pwmOneDelta++;
#endif
    }
    // at this point next must be >= 1 becasue setting a reload of 0 will freeze the timer
    // update the reload time. We need to ensure that it is >= the current tick count
    for(;;)
    {
        // Clear any pending interrupt
        __HAL_TIM_CLEAR_IT(timerHandle, TIM_IT_UPDATE);
        // set new target
        __HAL_TIM_SET_AUTORELOAD(timerHandle, next);
        uint16_t curTick = __HAL_TIM_GET_COUNTER(timerHandle);
        if (next >= curTick) break;
//        if (next > curTick) break;
#ifdef PWM_DEBUG
        pwmAdjust++;
#endif
        next = curTick;
    }
    // The actual elapsed time for a autoreload of n is n+1
    baseDelta = next + 1;
#ifdef PWM_DEBUG
    const uint32_t dt = dbtimer.getCount() - startTime;
    if (dt < pwmMinTime)
        pwmMinTime = dt;
    else if (dt > pwmMaxTime)
        pwmMaxTime = dt;
#endif
}

static void initTimer() noexcept
{
#ifdef PWM_DEBUG
{
	uint32_t preScale = dbtimer.getTimerClkFreq()/1000000;
	dbtimer.setPrescaleFactor(preScale);
	dbtimer.setOverflow(0, TICK_FORMAT);
    dbtimer.resume();
}
#endif
    for(uint32_t i = 0; i < MaxPWMChannels; i++)
        States[i].enabled = false;
    uint32_t preScale = SPWMTimer.getTimerClkFreq()/1000000;
    //debugPrintf("ST base freq %d setting presacle %d\n", static_cast<int>(SPWMTimer.getTimerClkFreq()), static_cast<int>(preScale));
    SPWMTimer.setPrescaleFactor(preScale);
    SPWMTimer.setOverflow(0xffff, TICK_FORMAT);
    // init hardware and interrupts
    timerHandle = SPWMTimer.getHandle();
    __HAL_TIM_SET_COUNTER(timerHandle, 0);
    NVIC_EnableIRQ(TIM7_IRQn);
    timerReady = true;
    SPWMTimer.resume();
}


void SPWMDiagnostics(const StringRef& reply)
{
#ifdef PWM_DEBUG
    reply.printf("\nErr: %d/%d; Acc %d; OOB: %u; Pend: %u; ANeg %u; CNeg %u; Early %u; One %u Ints: %u; Calls %u; \nfast: %uuS; slow %uuS adj %u vbd %u range %u badval %d\n", (int)pwmMinErr, (int)pwmMaxErr, (int)pwmAccErr, (unsigned)pwmOOB, (unsigned)pwmPending, (unsigned)pwmNegADelta, (unsigned)pwmNegCDelta, (unsigned)pwmEarly, (unsigned)pwmOneDelta, (unsigned)pwmInts, (unsigned)pwmCalls, (unsigned)pwmMinTime, (unsigned)pwmMaxTime, (unsigned)pwmAdjust, (unsigned)pwmVBigDelta, (unsigned)pwmBadRange, pwmBadVal);
    pwmMinTime = UINT32_MAX;
    pwmMaxTime = 0;
    pwmInts = 0;
    pwmCalls = 0;
    pwmAdjust = 0;
    pwmBigDelta = 0;
    pwmBadRange = 0;
    pwmPending = 0;
    pwmNegADelta = 0;
    pwmNegCDelta = 0;
    pwmEarly = 0;
    pwmOneDelta = 0;
    pwmMaxErr = -0x7fffffff;
    pwmMinErr = 0x7fffffff;
    pwmOOB = 0;
    pwmVBigDelta = 0;
#else
    reply.copy("");
#endif
}

SoftwarePWM::SoftwarePWM() noexcept : channel(-1), period(0xffffffff)
{
} 

void SoftwarePWM::free() noexcept
{
    if (channel >= 0)
        disable(channel);
    channel = -1;
    period = 0xffffffff;
}


HybridPWMBase *SoftwarePWM::allocate(Pin pin, uint32_t freq, float value) noexcept
{
    //debugPrintf("SWPWM allocate pin %x, freq %d\n", static_cast<int>(pin), static_cast<int>(freq));
    if (!timerReady)
        initTimer();
    for(uint32_t i = 0; i < MaxPWMChannels; i++)
        if (PWMChans[i].period == 0xffffffff)
        {
            PWMChans[i].period = (freq!=0)?(1000000/freq):0;
            PWMChans[i].setValue(pin, value);
            return &PWMChans[i];
        }

    return nullptr;
}

void SoftwarePWM::setValue(Pin pin, float value) noexcept
{
    if (period == 0)
    {
        SetPinMode(pin, (value < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
        return;
    }
    uint32_t onTime = (uint32_t)(period * value);
    if(onTime < MinimumInterruptDeltaUS){ onTime = 0; }
    if(onTime > (period-MinimumInterruptDeltaUS)){ onTime = period; }
    if (onTime == 0)
    {
        //debugPrintf("pin %d chan %d off\n", pin, channel);
        if (channel >= 0)
        {
            disable(channel);
            channel = -1;
        }
        SetPinMode(pin, OUTPUT_LOW);
    }
    else if (onTime == period)
    {
        if (channel >= 0)
        {
            disable(channel);
            channel = -1;
        }
        //debugPrintf("pin %d chan %d on\n", pin, channel);
        SetPinMode(pin, OUTPUT_HIGH);

    }
    else
    {
        if (channel < 0)
        {
            channel = enable(pin, onTime, period - onTime);
        }
        else
            adjustOnOffTime(channel, onTime, period - onTime);
    }
}

void SoftwarePWM::setValue(float value) noexcept
{
    setValue(pwmPin->pin, value);
}

void SoftwarePWM::appendStatus(const StringRef& reply) noexcept
{
    if (channel >= 0)
    {
        uint32_t now = baseTime + SPWMTimer.getCount();
        reply.catf(" channel %d next %d on %u off %u", (int)channel, (int)(States[channel].nextEvent - now), (unsigned)States[channel].onOffTimes[States[channel].onOffBuffer][0], (unsigned)States[channel].onOffTimes[States[channel].onOffBuffer][1]);
    }
    else
        reply.catf(" period %d", static_cast<int>(period));   
}