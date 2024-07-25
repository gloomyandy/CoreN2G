//author: Andy
#if USE_UART0 || USE_UART1 || USE_UART2
#include <CoreImp.h>
#include <CoreNotifyIndices.h>
#include "ConfigurableUART.h"
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

#if USE_UART0
TASKMEM ConfigurableUART UART_Slot0;
#endif
#if USE_UART1
TASKMEM ConfigurableUART UART_Slot1;
#endif
#if USE_UART2
TASKMEM ConfigurableUART UART_Slot2;
#endif



ConfigurableUART::ConfigurableUART() noexcept
{
    uart = nullptr;
    prio = 0;
    txWaitingTask = nullptr;
}


int8_t ConfigurableUART::GetUARTPortNumber() noexcept
{
    return get_port_number();
}

bool ConfigurableUART::Configure(Pin rx, Pin tx) noexcept
{
    //Find the UART based on the confgured Pins
    void* rxDev = pinmap_peripheral(rx, PinMap_UART_RX);
    void* txDev = pinmap_peripheral(tx, PinMap_UART_TX);

    if (rxDev != nullptr && (rxDev == txDev))
    {
        pin_rx = rx;
        pin_tx = tx;
        return true;
    }
    return false;    
}

void ConfigurableUART::begin(uint32_t baud, uint8_t config) noexcept
{
    uint32_t databits = 0;
    uint32_t stopbits = 0;
    uint32_t parity = 0;

    // Manage databits
    switch (config & 0x07) 
    {
    case 0x02:
        databits = 6;
        break;
    case 0x04:
        databits = 7;
        break;
    case 0x06:
        databits = 8;
        break;
    default:
        databits = 0;
        break;
    }

    if ((config & 0x30) == 0x30) 
    {
        parity = UART_PARITY_ODD;
        databits++;
    } 
    else if ((config & 0x20) == 0x20) 
    {
        parity = UART_PARITY_EVEN;
        databits++;
    } 
    else 
    {
        parity = UART_PARITY_NONE;
    }

    if ((config & 0x08) == 0x08) 
    {
        stopbits = UART_STOPBITS_2;
    } 
    else 
    {
        stopbits = UART_STOPBITS_1;
    }

    switch (databits) 
    {
    #ifdef UART_WORDLENGTH_7B
        case 7:
        databits = UART_WORDLENGTH_7B;
        break;
    #endif
        case 8:
        databits = UART_WORDLENGTH_8B;
        break;
        case 9:
        databits = UART_WORDLENGTH_9B;
        break;
        default:
        case 0:
        Error_Handler();
        break;
    }

    init( (uint32_t)baud, databits, parity, stopbits);
    if (uart != nullptr)
    {
        rx_head = 0;
        rx_tail = 0;
        tx_head = 0;
        tx_tail = 0;
        start_rx();
    }
}

void ConfigurableUART::begin(uint32_t baud) noexcept
{
    begin(baud, SERIAL_8N1);
}

void ConfigurableUART::end(void) noexcept
{
    if (uart != nullptr)
    {
        // wait for transmission of outgoing data
        flush();

        deinit();

        // clear any received data
        rx_head = rx_tail = 0;
    }
}

int ConfigurableUART::read(void) noexcept
{
    if (uart != nullptr)
    {
        // if the head isn't ahead of the tail, we don't have any characters
        if (rx_head == rx_tail) 
        {
            return -1;
        } 
        else 
        {
            unsigned char c = rx_buff[rx_tail];
            rx_tail = (rx_tail + 1) % SERIAL_RX_BUFFER_SIZE;
            return c;
        }
    }

    return -1;
}

int ConfigurableUART::peek(void) noexcept
{
    if (uart != nullptr)
    {
        if (rx_head == rx_tail) 
        {
            return -1;
        } 
        else 
        {
            return rx_buff[rx_tail];
        }
    }
    
    return -1;
}

int ConfigurableUART::available(void) noexcept
{
    if (uart != nullptr)
    {
        return rx_available();
    }
    return 0;
}

int ConfigurableUART::availableForWrite(void) noexcept
{
    if (uart != nullptr)
    {
        return tx_available();
    }
    return 0;
}


size_t ConfigurableUART::canWrite() noexcept
{
    if (uart != nullptr)
    {
        return availableForWrite();
    }
    return 0;
}

size_t ConfigurableUART::write(const uint8_t c) noexcept
{
    if (uart != nullptr)
    {
        uint32_t i = (tx_head + 1) % SERIAL_TX_BUFFER_SIZE;
        // If the output buffer is full, there's nothing for it other than to
        // wait for the interrupt handler to empty it a bit
        while (i == tx_tail) 
        {
            // nop, the interrupt handler will free up space for us
#ifdef RTOS
            txWaitingTask = RTOSIface::GetCurrentTask();
            TaskBase::TakeIndexed(NotifyIndices::UartTx, 50);
#endif
        }

        tx_buff[tx_head] = c;
        tx_head = i;

        if (!serial_tx_active()) 
        {
            start_tx();
        }

        return 1;
    }

    return 1;
}


size_t ConfigurableUART::writeBlock(const uint8_t *buffer, size_t size) noexcept
{
    uint32_t head = tx_head;
    uint32_t tail = tx_tail;
    size_t toCopy = (SERIAL_TX_BUFFER_SIZE - 1 - head + tail) % SERIAL_TX_BUFFER_SIZE;

    if (toCopy > size) toCopy = size;
    if (toCopy > 0)
    {
        size_t toCopyNext;
        if (head >= tail)
        {
            size_t toCopyFirst = SERIAL_TX_BUFFER_SIZE - head;
            if (toCopy < toCopyFirst)
            {
                memcpy(&(tx_buff[head]), buffer, toCopy);
                tx_head = head + toCopy;
                return toCopy;
            }
            memcpy(&(tx_buff[head]), buffer, toCopyFirst);
            head = 0;
            toCopyNext = toCopy - toCopyFirst;
            buffer += toCopyFirst;
        }
        else
            toCopyNext = toCopy;
        memcpy(&(tx_buff[head]), buffer, toCopyNext);
        tx_head = head + toCopyNext;
    }
    return toCopy;
}

size_t ConfigurableUART::write(const uint8_t *buffer, size_t size) noexcept
{
    if (uart != nullptr)
    {
        size_t ret = size;
        for(;;)
        {
            size_t len = writeBlock(buffer, size);
            size -= len;
            buffer += len;
            if (len && !serial_tx_active())
                start_tx();
            if (size == 0)
            {
                break;
            }
#ifdef RTOS
            txWaitingTask = RTOSIface::GetCurrentTask();
            TaskBase::TakeIndexed(NotifyIndices::UartTx, 50);
#endif
        }
        return ret;
    }

    return size;
}

void ConfigurableUART::flush(void) noexcept
{
    if (uart != nullptr)
    {
        // wait for the buffer to empty
        while ((tx_head != tx_tail)) 
        {
            // nop, the interrupt handler will free up space for us
        }
        // and for the hardware to complete sending
        while (serial_tx_active()) 
        {
        }
   }
}


void ConfigurableUART::setInterruptPriority(uint32_t priority) noexcept
{
    prio = priority;
    if (uart != nullptr)
    {
        set_interrupt_priority( priority);
    }
}

uint32_t ConfigurableUART::getInterruptPriority() noexcept
{
#if 0
// FIXME
    if (uart != nullptr)
    {
        return 0;
    }
#endif   
    return 0;
}

bool ConfigurableUART::IsConnected() noexcept
{
    if (uart != nullptr) return true;
    return false;
}


// FIXME we should probbaly implement the call back for this!
ConfigurableUART::InterruptCallbackFn ConfigurableUART::SetInterruptCallback(InterruptCallbackFn f) noexcept
{
	InterruptCallbackFn ret = interruptCallback;
	interruptCallback = f;
	return ret;
}

// Get and clear the errors
ConfigurableUART::Errors ConfigurableUART::GetAndClearErrors() noexcept
{
	Errors errs;
    flush();
    errs.uartOverrun = 0;
    if  (uart != nullptr)
    {
        errs.bufferOverrun = rx_full;
        errs.framing = hw_error;
        hw_error = rx_full = 0;
    }
    else
    {
        errs.bufferOverrun = 0;
        errs.framing = 0;
    }
	return errs;
}


// low level implementation
typedef enum {
#if defined(USART1_BASE)
    UART1_INDEX,
#endif
#if defined(USART2_BASE)
    UART2_INDEX,
#endif
#if defined(USART3_BASE)
    UART3_INDEX,
#endif
#if defined(UART4_BASE) || defined(USART4_BASE)
    UART4_INDEX,
#endif
#if defined(UART5_BASE) || defined(USART5_BASE)
    UART5_INDEX,
#endif
#if defined(USART6_BASE)
    UART6_INDEX,
#endif
#if defined(UART7_BASE) || defined(USART7_BASE)
    UART7_INDEX,
#endif
#if defined(UART8_BASE) || defined(USART8_BASE)
    UART8_INDEX,
#endif
#if defined(UART9_BASE)
    UART9_INDEX,
#endif
#if defined(UART10_BASE)
    UART10_INDEX,
#endif
#if defined(LPUART1_BASE)
    LPUART1_INDEX,
#endif
    UART_NUM
} index_t;

static ConfigurableUART *handlers[UART_NUM] = {nullptr};

void ConfigurableUART::init(uint32_t baudrate, uint32_t databits, uint32_t parity, uint32_t stopbits) noexcept
{
    UART_HandleTypeDef *huart = &(handle);
    uint8_t index = 0;
    /* Determine the U(S)ART peripheral to use (USART1, USART2, ...) */
    void *tx = pinmap_peripheral(pin_tx, PinMap_UART_TX);
    void *rx = pinmap_peripheral(pin_rx, PinMap_UART_RX);

    /* Pins Rx/Tx must not be NP */
    if (rx == NP || tx == NP) {
        debugPrintf("ERROR: at least one UART pin has no peripheral\n");
        return;
    }
    hw_error = rx_full = 0;
 
    /*
     * Get the peripheral name (USART1, USART2, ...) from the pin
     * and assign it to the ect
     */
    uart = (USART_TypeDef *)pinmap_merge_peripheral(tx, rx);

    if (uart == NP) {
        debugPrintf("ERROR: U(S)ART pins mismatch\n");
        return;
    }

    /* Enable USART clock */
#if defined(USART1_BASE)
    else if (uart == USART1) {
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_ENABLE();
        index = UART1_INDEX;
        irq = USART1_IRQn;
    }
#endif
#if defined(USART2_BASE)
    else if (uart == USART2) {
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        __HAL_RCC_USART2_CLK_ENABLE();
        index = UART2_INDEX;
        irq = USART2_IRQn;
    }
#endif
#if defined(USART3_BASE)
    else if (uart == USART3) {
        __HAL_RCC_USART3_FORCE_RESET();
        __HAL_RCC_USART3_RELEASE_RESET();
        __HAL_RCC_USART3_CLK_ENABLE();
        index = UART3_INDEX;
        irq = USART3_IRQn;
    }
#endif
#if defined(UART4_BASE)
    else if (uart == UART4) {
        __HAL_RCC_UART4_FORCE_RESET();
        __HAL_RCC_UART4_RELEASE_RESET();
        __HAL_RCC_UART4_CLK_ENABLE();
        index = UART4_INDEX;
        irq = UART4_IRQn;
    }
#elif defined(USART4_BASE)
    else if (uart == USART4) {
        __HAL_RCC_USART4_FORCE_RESET();
        __HAL_RCC_USART4_RELEASE_RESET();
        __HAL_RCC_USART4_CLK_ENABLE();
        index = UART4_INDEX;
        irq = USART4_IRQn;
    }
#endif
#if defined(UART5_BASE)
    else if (uart == UART5) {
        __HAL_RCC_UART5_FORCE_RESET();
        __HAL_RCC_UART5_RELEASE_RESET();
        __HAL_RCC_UART5_CLK_ENABLE();
        index = UART5_INDEX;
        irq = UART5_IRQn;
    }
#elif defined(USART5_BASE)
    else if (uart == USART5) {
        __HAL_RCC_USART5_FORCE_RESET();
        __HAL_RCC_USART5_RELEASE_RESET();
        __HAL_RCC_USART5_CLK_ENABLE();
        index = UART5_INDEX;
        irq = USART5_IRQn;
    }
#endif
#if defined(USART6_BASE)
    else if (uart == USART6) {
        __HAL_RCC_USART6_FORCE_RESET();
        __HAL_RCC_USART6_RELEASE_RESET();
        __HAL_RCC_USART6_CLK_ENABLE();
        index = UART6_INDEX;
        irq = USART6_IRQn;
    }
#endif
#if defined(LPUART1_BASE)
    else if (uart == LPUART1) {
        __HAL_RCC_LPUART1_FORCE_RESET();
        __HAL_RCC_LPUART1_RELEASE_RESET();
        __HAL_RCC_LPUART1_CLK_ENABLE();
        index = LPUART1_INDEX;
        irq = LPUART1_IRQn;
    }
#endif
#if defined(UART7_BASE)
    else if (uart == UART7) {
        __HAL_RCC_UART7_FORCE_RESET();
        __HAL_RCC_UART7_RELEASE_RESET();
        __HAL_RCC_UART7_CLK_ENABLE();
        index = UART7_INDEX;
        irq = UART7_IRQn;
    }
#elif defined(USART7_BASE)
    else if (uart == USART7) {
        __HAL_RCC_USART7_FORCE_RESET();
        __HAL_RCC_USART7_RELEASE_RESET();
        __HAL_RCC_USART7_CLK_ENABLE();
        index = UART7_INDEX;
        irq = USART7_IRQn;
    }
#endif
#if defined(UART8_BASE)
    else if (uart == UART8) {
        __HAL_RCC_UART8_FORCE_RESET();
        __HAL_RCC_UART8_RELEASE_RESET();
        __HAL_RCC_UART8_CLK_ENABLE();
        index = UART8_INDEX;
        irq = UART8_IRQn;
    }
#elif defined(USART8_BASE)
    else if (uart == USART8) {
        __HAL_RCC_USART8_FORCE_RESET();
        __HAL_RCC_USART8_RELEASE_RESET();
        __HAL_RCC_USART8_CLK_ENABLE();
        index = UART8_INDEX;
        irq = USART8_IRQn;
    }
#endif
#if defined(UART9_BASE)
    else if (uart == UART9) {
        __HAL_RCC_UART9_FORCE_RESET();
        __HAL_RCC_UART9_RELEASE_RESET();
        __HAL_RCC_UART9_CLK_ENABLE();
        index = UART9_INDEX;
        irq = UART9_IRQn;
    }
#endif
#if defined(UART10_BASE)
    else if (uart == UART10) {
        __HAL_RCC_UART10_FORCE_RESET();
        __HAL_RCC_UART10_RELEASE_RESET();
        __HAL_RCC_UART10_CLK_ENABLE();
        index = UART10_INDEX;
        irq = UART10_IRQn;
    }
#endif
    else
    {
        debugPrintf("UART not found\n");
        uart = nullptr;
        return;
    }
#if defined(STM32F091xC) || defined (STM32F098xx)
    /* Enable SYSCFG Clock */
    /* Required to get SYSCFG interrupt status register */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
#endif

    /* Configure UART GPIO pins */
    pinmap_pinout(pin_tx, PinMap_UART_TX);
    pinmap_pinout(pin_rx, PinMap_UART_RX);

    /* Configure uart */
    handlers[index] = this;
    handle.Instance = (USART_TypeDef *)(uart);
    handle.Init.BaudRate = baudrate;
    handle.Init.WordLength = databits;
    handle.Init.StopBits = stopbits;
    handle.Init.Parity = parity;
    handle.Init.Mode = UART_MODE_TX_RX;
    handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handle.Init.OverSampling = UART_OVERSAMPLING_16;
#if (defined(STM32H7xx))
    handle.FifoMode = USART_FIFOMODE_DISABLE;
#endif
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F4xx)\
 && !defined(STM32L1xx)
    handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif
#ifdef UART_ONE_BIT_SAMPLE_DISABLE
    handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
#endif


    if (HAL_UART_Init(huart) != HAL_OK) {
        return;
    }
    if (prio != 0)
    {
        set_interrupt_priority(prio);
    }
    HAL_NVIC_EnableIRQ(irq);
}

void ConfigurableUART::deinit() noexcept
{
    /* Reset UART and disable clock */
    HAL_NVIC_DisableIRQ(irq);
#if defined(USART1_BASE)
    if (uart == USART1) {
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_DISABLE();
    }
#endif
#if defined(USART2_BASE)
    else if (uart == USART2) {
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        __HAL_RCC_USART2_CLK_DISABLE();
    }
#endif
#if defined(USART3_BASE)
    else if (uart == USART3) {
        __HAL_RCC_USART3_FORCE_RESET();
        __HAL_RCC_USART3_RELEASE_RESET();
        __HAL_RCC_USART3_CLK_DISABLE();
    }
#endif
#if defined(UART4_BASE)
    else if (uart == UART4) {
        __HAL_RCC_UART4_FORCE_RESET();
        __HAL_RCC_UART4_RELEASE_RESET();
        __HAL_RCC_UART4_CLK_DISABLE();
    }
#elif defined(USART4_BASE)
    else if (uart == USART4) {
        __HAL_RCC_USART4_FORCE_RESET();
        __HAL_RCC_USART4_RELEASE_RESET();
        __HAL_RCC_USART4_CLK_DISABLE();
    }
#endif
#if defined(UART5_BASE)
    else if (uart == UART5) {
        __HAL_RCC_UART5_FORCE_RESET();
        __HAL_RCC_UART5_RELEASE_RESET();
        __HAL_RCC_UART5_CLK_DISABLE();
    }
#elif defined(USART5_BASE)
    else if (uart == USART5) {
        __HAL_RCC_USART5_FORCE_RESET();
        __HAL_RCC_USART5_RELEASE_RESET();
        __HAL_RCC_USART5_CLK_DISABLE();
    }
#endif
#if defined(USART6_BASE)
    else if (uart == USART6) {
        __HAL_RCC_USART6_FORCE_RESET();
        __HAL_RCC_USART6_RELEASE_RESET();
        __HAL_RCC_USART6_CLK_DISABLE();
    }
#endif
#if defined(LPUART1_BASE)
    else if (uart == LPUART1) {
        __HAL_RCC_LPUART1_FORCE_RESET();
        __HAL_RCC_LPUART1_RELEASE_RESET();
        __HAL_RCC_LPUART1_CLK_DISABLE();
    }
#endif
#if defined(UART7_BASE)
    else if (uart == UART7) {
        __HAL_RCC_UART7_FORCE_RESET();
        __HAL_RCC_UART7_RELEASE_RESET();
        __HAL_RCC_UART7_CLK_DISABLE();
    }
#elif defined(USART7_BASE)
    else if (uart == USART7) {
        __HAL_RCC_USART7_FORCE_RESET();
        __HAL_RCC_USART7_RELEASE_RESET();
        __HAL_RCC_USART7_CLK_DISABLE();
    }
#endif
#if defined(UART8_BASE)
    else if (uart == UART8) {
        __HAL_RCC_UART8_FORCE_RESET();
        __HAL_RCC_UART8_RELEASE_RESET();
        __HAL_RCC_UART8_CLK_DISABLE();
    }
#elif defined(USART8_BASE)
    else if (uart == USART8) {
        __HAL_RCC_USART8_FORCE_RESET();
        __HAL_RCC_USART8_RELEASE_RESET();
        __HAL_RCC_USART8_CLK_DISABLE();
    }
#endif
#if defined(UART9_BASE)
    else if (uart == UART9) {
        __HAL_RCC_UART9_FORCE_RESET();
        __HAL_RCC_UART9_RELEASE_RESET();
        __HAL_RCC_UART9_CLK_DISABLE();
    }
#endif
#if defined(UART10_BASE)
    else if (uart == UART10) {
        __HAL_RCC_UART10_FORCE_RESET();
        __HAL_RCC_UART10_RELEASE_RESET();
        __HAL_RCC_UART10_CLK_DISABLE();
    }
#endif
    HAL_UART_DeInit(&handle);

}

int8_t ConfigurableUART::get_port_number() noexcept
{
#if defined(USART1_BASE)
    if (uart == USART1) {
        return 1;
    }
#endif
#if defined(USART2_BASE)
    else if (uart == USART2) {
        return 2;
 }
#endif
#if defined(USART3_BASE)
    else if (uart == USART3) {
        return 3;
    }
#endif
#if defined(UART4_BASE)
    else if (uart == UART4) {
        return 4;
    }
#elif defined(USART4_BASE)
    else if (uart == USART4) {
        return 4;
    }
#endif
#if defined(UART5_BASE)
    else if (uart == UART5) {
        return 5;
    }
#elif defined(USART5_BASE)
    else if (uart == USART5) {
        return 5;
    }
#endif
#if defined(USART6_BASE)
    else if (uart == USART6) {
        return 6;
    }
#endif
#if defined(LPUART1_BASE)
    else if (uart == LPUART1) {
        return 1;
    }
#endif
#if defined(UART7_BASE)
    else if (uart == UART7) {
        return 7;
    }
#elif defined(USART7_BASE)
    else if (uart == USART7) {
        return 7;
    }
#endif
#if defined(UART8_BASE)
    else if (uart == UART8) {
        return 8;
    }
#elif defined(USART8_BASE)
    else if (uart == USART8) {
        return 8;
    }
#endif
#if defined(UART9_BASE)
    else if (uart == UART9) {
        return 9;
    }
#endif
#if defined(UART10_BASE)
    else if (uart == UART10) {
        return 10;
    }
#endif
    return -1;
}

uint8_t ConfigurableUART::serial_rx_active() noexcept
{
    return ((HAL_UART_GetState(&handle) & HAL_UART_STATE_BUSY_RX) == HAL_UART_STATE_BUSY_RX);
}

uint8_t ConfigurableUART::serial_tx_active() noexcept
{
    return ((HAL_UART_GetState(&handle) & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_BUSY_TX);
}


void ConfigurableUART::start_rx() noexcept
{
    /* Exit if a reception is already on-going */
    if (serial_rx_active()) {
        return;
    }
    HAL_UART_Receive_IT(&handle, &(rx_buff[0]), SERIAL_RX_BUFFER_SIZE);
}

void ConfigurableUART::start_tx() noexcept
{
    if (!serial_tx_active() && tx_tail != tx_head) {
        HAL_UART_Transmit_IT(&handle, &tx_buff[tx_tail], 1);
    }
}

void ConfigurableUART::set_interrupt_priority( uint32_t priority) noexcept
{
    HAL_NVIC_SetPriority(irq, priority, 0);
}


uint32_t ConfigurableUART::rx_available() noexcept
{
    return ((uint32_t)(rx_head - rx_tail)) % SERIAL_RX_BUFFER_SIZE;
}

uint32_t ConfigurableUART::tx_available() noexcept
{
    return ((uint32_t)(tx_tail + SERIAL_TX_BUFFER_SIZE - 1 - tx_head)) % SERIAL_TX_BUFFER_SIZE;
}

inline void ConfigurableUART::UART_ErrorCallback() noexcept
{
    /* Restart receive interrupt after any error */
    hw_error++;
    if (!serial_rx_active()) {
        HAL_UART_Receive_IT(&handle, &(rx_buff[rx_head]), 1);
    }
}

inline HAL_StatusTypeDef ConfigurableUART::UART_Receive_IT() noexcept
{
#if STM32H7
    uint8_t val = (uint8_t)(handle.Instance->RDR & (uint8_t)0x00FF);
#else
    uint8_t val = (uint8_t)(handle.Instance->DR & (uint8_t)0x00FF);
#endif
    uint32_t pos = (rx_head + 1) % SERIAL_RX_BUFFER_SIZE;
    if (pos != rx_tail)
    {
        rx_buff[rx_head] = val;
        rx_head = pos;
    }
    else
        rx_full++;
    return HAL_OK;
}

inline HAL_StatusTypeDef ConfigurableUART::UART_Transmit_IT() noexcept
{
    // write the data
#if STM32H7
    handle.Instance->TDR = tx_buff[tx_tail];
#else
    handle.Instance->DR = tx_buff[tx_tail];
#endif
    // update write pointer
    tx_tail = (tx_tail + 1) % SERIAL_TX_BUFFER_SIZE;
    // do we have more to send?
    if (tx_tail == tx_head)
    {
        /* Disable the UART Transmit empty Interrupt */
        __HAL_UART_DISABLE_IT(&handle, UART_IT_TXE);

        /* Enable the UART Transmit Complete Interrupt */
        __HAL_UART_ENABLE_IT(&handle, UART_IT_TC);
    }
#if RTOS
    if (txWaitingTask != nullptr && tx_available() >= SERIAL_TX_BUFFER_SIZE/2)
    {
        TaskBase::GiveFromISR(txWaitingTask, NotifyIndices::UartTx);
        txWaitingTask = nullptr;
    }
#endif


    return HAL_OK;
}

inline HAL_StatusTypeDef ConfigurableUART::UART_EndTransmit_IT() noexcept
{
    // has more data arrived while we waited?
    if (tx_tail != tx_head)
    {
        // Yes so send the data
#if STM32H7
        handle.Instance->TDR = tx_buff[tx_tail];
#else
        handle.Instance->DR = tx_buff[tx_tail];
#endif
        // update write pointer
        tx_tail = (tx_tail + 1) % SERIAL_TX_BUFFER_SIZE;
        // do we have more to send?
        if (tx_tail != tx_head)
        {
            /* Enable the UART Transmit empty Interrupt */
            __HAL_UART_ENABLE_IT(&handle, UART_IT_TXE);
            __HAL_UART_DISABLE_IT(&handle, UART_IT_TC);
        }
        return HAL_OK;
    }
    // No data left to send
    /* Disable the UART Transmit Complete Interrupt */
    __HAL_UART_DISABLE_IT(&handle, UART_IT_TC);
    /* Tx process is ended, restore handle.gState to Ready */
    handle.gState = HAL_UART_STATE_READY;
    return HAL_OK;
}

void ConfigurableUART::UART_IRQHandler() noexcept
{
#if STM32H7
    uint32_t isrflags = READ_REG(handle.Instance->ISR);
    uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));
#else
    uint32_t isrflags = READ_REG(handle.Instance->SR);
    uint32_t errorflags = (isrflags & (uint32_t)(UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE));
#endif
    uint32_t cr1its = READ_REG(handle.Instance->CR1);
    uint32_t cr3its         = READ_REG(handle.Instance->CR3);

    /* If no error occurs */
    if (errorflags == RESET)
    {
        /* UART in mode Receiver -------------------------------------------------*/
        if (((isrflags & UART_FLAG_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
        {
            UART_Receive_IT();
            return;
        }
    }

    /* If some errors occur */
// FIXME: This is a mess different flags on H7/F4
#if STM32H7
    if ((errorflags != 0U)
            && ((((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != 0U) || ((cr1its & (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE | USART_CR1_RTOIE)) != 0U))))
    {
        /* UART parity error interrupt occurred -------------------------------------*/
        if (((isrflags & USART_ISR_PE) != 0U) && ((cr1its & USART_CR1_PEIE) != 0U))
        {
            __HAL_UART_CLEAR_FLAG(&handle, UART_CLEAR_PEF);
            handle.ErrorCode |= HAL_UART_ERROR_PE;
        }

        /* UART frame error interrupt occurred --------------------------------------*/
        if (((isrflags & USART_ISR_FE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
        {
            __HAL_UART_CLEAR_FLAG(&handle, UART_CLEAR_FEF);
            handle.ErrorCode |= HAL_UART_ERROR_FE;
        }

        /* UART noise error interrupt occurred --------------------------------------*/
        if (((isrflags & USART_ISR_NE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
        {
            __HAL_UART_CLEAR_FLAG(&handle, UART_CLEAR_NEF);
            handle.ErrorCode |= HAL_UART_ERROR_NE;
        }

        /* UART Over-Run interrupt occurred -----------------------------------------*/
        if (((isrflags & USART_ISR_ORE) != 0U) && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U) || ((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != 0U)))
        {
            __HAL_UART_CLEAR_FLAG(&handle, UART_CLEAR_OREF);
            handle.ErrorCode |= HAL_UART_ERROR_ORE;
        }

        /* UART Receiver Timeout interrupt occurred ---------------------------------*/
        if (((isrflags & USART_ISR_RTOF) != 0U) && ((cr1its & USART_CR1_RTOIE) != 0U))
        {
            __HAL_UART_CLEAR_FLAG(&handle, UART_CLEAR_RTOF);
            handle.ErrorCode |= HAL_UART_ERROR_RTO;
        }

        /* Call UART Error Call back function if need be ----------------------------*/
        if (handle.ErrorCode != HAL_UART_ERROR_NONE)
        {
            /* UART in mode Receiver --------------------------------------------------*/
            if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U)
                    && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U)
                            || ((cr3its & USART_CR3_RXFTIE) != 0U)))
            {
                UART_Receive_IT();
            }
            // record error and restart
            UART_ErrorCallback();
            handle.ErrorCode = HAL_UART_ERROR_NONE;
        }
        return;
    }
#else
    /* If some errors occur */
    if ((errorflags != RESET) && (((cr3its & USART_CR3_EIE) != RESET) || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
    {
        /* UART parity error interrupt occurred ----------------------------------*/
        if (((isrflags & USART_SR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
        {
            handle.ErrorCode |= HAL_UART_ERROR_PE;
        }

        /* UART noise error interrupt occurred -----------------------------------*/
        if (((isrflags & USART_SR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            handle.ErrorCode |= HAL_UART_ERROR_NE;
        }

        /* UART frame error interrupt occurred -----------------------------------*/
        if (((isrflags & USART_SR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            handle.ErrorCode |= HAL_UART_ERROR_FE;
        }

        /* UART Over-Run interrupt occurred --------------------------------------*/
        if (((isrflags & USART_SR_ORE) != RESET) && (((cr1its & USART_CR1_RXNEIE) != RESET) || ((cr3its & USART_CR3_EIE) != RESET)))
        {
            handle.ErrorCode |= HAL_UART_ERROR_ORE;
        }

        /* Call UART Error Call back function if need be --------------------------*/
        if (handle.ErrorCode != HAL_UART_ERROR_NONE)
        {
            /* UART in mode Receiver -----------------------------------------------*/
            if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
            {
                UART_Receive_IT();
            }

            // record error and restart
            UART_ErrorCallback();
            handle.ErrorCode = HAL_UART_ERROR_NONE;
        }
        return;
    } /* End if some error occurs */
#endif
    /* UART in mode Transmitter ------------------------------------------------*/
    if (((isrflags & UART_FLAG_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
    {
        UART_Transmit_IT();
        return;
    }

    /* UART in mode Transmitter end --------------------------------------------*/
    if (((isrflags & UART_FLAG_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
    {
        UART_EndTransmit_IT();
        return;
    }
}

extern "C" {
#if defined(USART1_BASE)
void USART1_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
    handlers[UART1_INDEX]->UART_IRQHandler();
}
#endif

#if defined(USART2_BASE)
void USART2_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
    handlers[UART2_INDEX]->UART_IRQHandler();
}
#endif

#if defined(USART3_BASE)
void USART3_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
#if defined(STM32F091xC) || defined (STM32F098xx)
    if (__HAL_GET_PENDING_IT(HAL_ITLINE_USART3) != RESET) {
        handlers[UART3_INDEX]->UART_IRQHandler();
    }
    if (__HAL_GET_PENDING_IT(HAL_ITLINE_USART4) != RESET) {
        handlers[UART4_INDEX]->UART_IRQHandler();
    }
    if (__HAL_GET_PENDING_IT(HAL_ITLINE_USART5) != RESET) {
        handlers[UART5_INDEX]->UART_IRQHandler();
    }
    if (__HAL_GET_PENDING_IT(HAL_ITLINE_USART6) != RESET) {
        handlers[UART6_INDEX]->UART_IRQHandler();
    }
    if (__HAL_GET_PENDING_IT(HAL_ITLINE_USART7) != RESET) {
        handlers[UART7_INDEX]->UART_IRQHandler();
    }
    if (__HAL_GET_PENDING_IT(HAL_ITLINE_USART8) != RESET) {
        handlers[UART8_INDEX]->UART_IRQHandler();
    }
#else
    if (handlers[UART3_INDEX] != NULL) {
        handlers[UART3_INDEX]->UART_IRQHandler();
    }
#if defined(STM32F0xx)
    /* USART3_4_IRQn */
    if (handlers[UART4_INDEX] != NULL) {
        handlers[UART4_INDEX]->UART_IRQHandler();
    }
#if defined(STM32F030xC)
    if (handlers[UART5_INDEX] != NULL) {
        handlers[UART5_INDEX]->UART_IRQHandler();
    }
    if (handlers[UART6_INDEX] != NULL) {
        handlers[UART6_INDEX]->UART_IRQHandler();
    }
#endif /* STM32F030xC */
#endif /* STM32F0xx */
#endif /* STM32F091xC || STM32F098xx */
}
#endif

#if defined(UART4_BASE)
void UART4_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(UART4_IRQn);
    handlers[UART4_INDEX]->UART_IRQHandler();
}
#endif

#if defined(STM32L0xx)
#if defined(USART4_BASE) || defined(USART5_BASE)
void USART4_5_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART4_IRQn);
    if (handlers[UART4_INDEX] != NULL) {
        handlers[UART4_INDEX]->UART_IRQHandler();
    }
    if (handlers[UART5_INDEX] != NULL) {
        handlers[UART5_INDEX]->UART_IRQHandler();
    }
}
#endif
#endif

#if defined(UART5_BASE)
void UART5_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(UART5_IRQn);
    handlers[UART5_INDEX]->UART_IRQHandler();
}
#endif

#if defined(USART6_BASE) && !defined(STM32F0xx)
void USART6_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART6_IRQn);
    handlers[UART6_INDEX]->UART_IRQHandler();
}
#endif

#if defined(LPUART1_BASE)
void LPUART1_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);
    handlers[LPUART1_INDEX]->UART_IRQHandler();
}
#endif

#if defined(UART7_BASE)
void UART7_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(UART7_IRQn);
    handlers[UART7_INDEX]->UART_IRQHandler();
}
#endif

#if defined(UART8_BASE)
void UART8_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(UART8_IRQn);
    handlers[UART8_INDEX]->UART_IRQHandler();
}
#endif

#if defined(UART9_BASE)
void UART9_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(UART9_IRQn);
    handlers[UART9_INDEX]->UART_IRQHandler();
}
#endif

#if defined(UART10_BASE)
void UART10_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(UART10_IRQn);
    handlers[UART10_INDEX]->UART_IRQHandler();
}
#endif
}
#endif
