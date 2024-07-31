/*
  Copyright (c) 2017 Daniel Fekete

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  Copyright (c) 2019 STMicroelectronics
  Heavily modified for RRF by Andy (c) 2024
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HARDWARETIMER_H_
#define HARDWARETIMER_H_

/* Includes ------------------------------------------------------------------*/

#ifdef HAL_TIM_MODULE_ENABLED
#define UNKNOWN_TIMER 0xffff
#define  TIMER_CHANNELS 4    // channel5 and channel 6 are not considered here has they don't have gpio output and they don't have interrupt
#define INVALID_ID 
typedef enum {
  TIMER_DISABLED,
  // Output Compare
  TIMER_OUTPUT_COMPARE,                   // == TIM_OCMODE_TIMING           no output, useful for only-interrupt
  TIMER_OUTPUT_COMPARE_ACTIVE,            // == TIM_OCMODE_ACTIVE           pin is set high when counter == channel compare
  TIMER_OUTPUT_COMPARE_INACTIVE,          // == TIM_OCMODE_INACTIVE         pin is set low when counter == channel compare
  TIMER_OUTPUT_COMPARE_TOGGLE,            // == TIM_OCMODE_TOGGLE           pin toggles when counter == channel compare
  TIMER_OUTPUT_COMPARE_PWM1,              // == TIM_OCMODE_PWM1             pin high when counter < channel compare, low otherwise
  TIMER_OUTPUT_COMPARE_PWM2,              // == TIM_OCMODE_PWM2             pin low when counter < channel compare, high otherwise
  TIMER_OUTPUT_COMPARE_FORCED_ACTIVE,     // == TIM_OCMODE_FORCED_ACTIVE    pin always high
  TIMER_OUTPUT_COMPARE_FORCED_INACTIVE,   // == TIM_OCMODE_FORCED_INACTIVE  pin always low

  //Input capture
  TIMER_INPUT_CAPTURE_RISING,             // == TIM_INPUTCHANNELPOLARITY_RISING
  TIMER_INPUT_CAPTURE_FALLING,            // == TIM_INPUTCHANNELPOLARITY_FALLING
  TIMER_INPUT_CAPTURE_BOTHEDGE,           // == TIM_INPUTCHANNELPOLARITY_BOTHEDGE

  // Used 2 channels for a single pin. One channel in TIM_INPUTCHANNELPOLARITY_RISING another channel in TIM_INPUTCHANNELPOLARITY_FALLING.
  // Channels must be used by pair: CH1 with CH2, or CH3 with CH4
  // This mode is very useful for Frequency and Dutycycle measurement
  TIMER_INPUT_FREQ_DUTY_MEASUREMENT,

  TIMER_NOT_USED = 0xFF  // This must be the last item of this enum
} TimerModes_t;

typedef enum {
  TICK_FORMAT, // default
  MICROSEC_FORMAT,
  HERTZ_FORMAT,
} TimerFormat_t;

typedef enum {
  TICK_COMPARE_FORMAT, // default
  MICROSEC_COMPARE_FORMAT,
  HERTZ_COMPARE_FORMAT,
  PERCENT_COMPARE_FORMAT,  // used for Dutycycle
  RESOLUTION_8B_COMPARE_FORMAT,  // used for Dutycycle: [0.. 255]
  RESOLUTION_12B_COMPARE_FORMAT  // used for Dutycycle: [0.. 4095]
} TimerCompareFormat_t;

#ifdef __cplusplus
/* Class --------------------------------------------------------*/
class HardwareTimer {
  public:
    HardwareTimer(TIM_TypeDef *instance) noexcept;
    ~HardwareTimer() noexcept;  // destructor

    void pause(void) noexcept;  // Pause counter and all output channels
    void resume(void) noexcept; // Resume counter and all output channels

    void setPrescaleFactor(uint32_t prescaler) noexcept; // set prescaler register (which is factor value - 1)

    void setOverflow(uint32_t val, TimerFormat_t format = TICK_FORMAT) noexcept; // set AutoReload register depending on format provided

    void setCount(uint32_t val, TimerFormat_t format = TICK_FORMAT) noexcept; // set timer counter to value 'val' depending on format provided
    uint32_t getCount(TimerFormat_t format = TICK_FORMAT) noexcept;  // return current counter value of timer depending on format provided

    void setMode(uint32_t channel, TimerModes_t mode, PinName pin = NC) noexcept; // Configure timer channel with specified mode on specified pin if available

    void setCaptureCompare(uint32_t channel, uint32_t compare, TimerCompareFormat_t format = TICK_COMPARE_FORMAT) noexcept;  // set Compare register value of specified channel depending on format provided

    uint32_t getTimerClkFreq() noexcept;  // return timer clock frequency in Hz.

    TIM_HandleTypeDef* getHandle() noexcept;

    uint32_t getTimerId() noexcept;
  private:
    TIM_HandleTypeDef handle;
    uint8_t OCMode[TIMER_CHANNELS];
    int getChannel(uint32_t channel);
    void resumeChannel(uint32_t channel);
};

#endif /* __cplusplus */

#endif  // HAL_TIM_MODULE_ENABLED
#endif  // HARDWARETIMER_H_