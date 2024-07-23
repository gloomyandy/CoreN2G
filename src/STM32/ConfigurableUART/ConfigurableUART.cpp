//author: Andy
#if USE_UART0 || USE_UART1 || USE_UART2
#include <CoreImp.h>
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
    _serial.index = -1;
}


int8_t ConfigurableUART::GetUARTPortNumber() noexcept
{
    return uart_get_port_number(&_serial);
}

bool ConfigurableUART::Configure(Pin rx, Pin tx) noexcept
{
    //Find the UART based on the confgured Pins
    void* rxDev = pinmap_peripheral(rx, PinMap_UART_RX);
    void* txDev = pinmap_peripheral(tx, PinMap_UART_TX);

    if (rxDev != nullptr && (rxDev == txDev))
    {
        _serial.pin_rx = rx;
        _serial.pin_tx = tx;
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

    uart_init(&_serial, (uint32_t)baud, databits, parity, stopbits);
    if (_serial.index >= 0)
    {
        _serial.rx_head = 0;
        _serial.rx_tail = 0;
        _serial.tx_head = 0;
        _serial.tx_tail = 0;
        uart_start_rx(&_serial);
    }
}

void ConfigurableUART::begin(uint32_t baud) noexcept
{
    begin(baud, SERIAL_8N1);
}

void ConfigurableUART::end(void) noexcept
{
    if (_serial.index >= 0)
    {
        // wait for transmission of outgoing data
        flush();

        uart_deinit(&_serial);

        // clear any received data
        _serial.rx_head = _serial.rx_tail = 0;
    }
}

int ConfigurableUART::read(void) noexcept
{
    if (_serial.index >= 0)
    {
        // if the head isn't ahead of the tail, we don't have any characters
        if (_serial.rx_head == _serial.rx_tail) 
        {
            return -1;
        } 
        else 
        {
            unsigned char c = _serial.rx_buff[_serial.rx_tail];
            _serial.rx_tail = (_serial.rx_tail + 1) % SERIAL_RX_BUFFER_SIZE;
            return c;
        }
    }

    return -1;
}

int ConfigurableUART::peek(void) noexcept
{
    if (_serial.index >= 0)
    {
        if (_serial.rx_head == _serial.rx_tail) 
        {
            return -1;
        } 
        else 
        {
            return _serial.rx_buff[_serial.rx_tail];
        }
    }
    
    return -1;
}

int ConfigurableUART::available(void) noexcept
{
    if (_serial.index >= 0)
    {
        return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _serial.rx_head - _serial.rx_tail)) % SERIAL_RX_BUFFER_SIZE;
    }
    return 0;
}

int ConfigurableUART::availableForWrite(void) noexcept
{
    if (_serial.index >= 0)
    {
        uint32_t head = _serial.tx_head;
        uint32_t tail = _serial.tx_tail;

        if (head >= tail) 
        {
            return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
        }
        return tail - head - 1;
    }
    return 0;
}


size_t ConfigurableUART::canWrite() noexcept
{
    if (_serial.index >= 0)
    {
        return availableForWrite();
    }
    return 0;
}

size_t ConfigurableUART::write(const uint8_t c) noexcept
{
    if (_serial.index >= 0)
    {
        uint32_t i = (_serial.tx_head + 1) % SERIAL_TX_BUFFER_SIZE;
        // If the output buffer is full, there's nothing for it other than to
        // wait for the interrupt handler to empty it a bit
        while (i == _serial.tx_tail) 
        {
            // nop, the interrupt handler will free up space for us
        }

        _serial.tx_buff[_serial.tx_head] = c;
        _serial.tx_head = i;

        if (!serial_tx_active(&_serial)) 
        {
            uart_start_tx(&_serial);
        }

        return 1;
    }

    return 1;
}


size_t ConfigurableUART::writeBlock(const uint8_t *buffer, size_t size) noexcept
{
    uint32_t head = _serial.tx_head;
    uint32_t tail = _serial.tx_tail;
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
                memcpy(&(_serial.tx_buff[head]), buffer, toCopy);
                _serial.tx_head = head + toCopy;
                return toCopy;
            }
            memcpy(&(_serial.tx_buff[head]), buffer, toCopyFirst);
            head = 0;
            toCopyNext = toCopy - toCopyFirst;
            buffer += toCopyFirst;
        }
        else
            toCopyNext = toCopy;
        memcpy(&(_serial.tx_buff[head]), buffer, toCopyNext);
        _serial.tx_head = head + toCopyNext;
    }
    return toCopy;
}

size_t ConfigurableUART::write(const uint8_t *buffer, size_t size) noexcept
{
    if (_serial.index >= 0)
    {
        size_t ret = size;
        while (size > 0)
        {
            size_t len = writeBlock(buffer, size);
            size -= len;
            buffer += len;
            if (len && !serial_tx_active(&_serial))
                uart_start_tx(&_serial);    
        }
        return ret;
    }

    return size;
}

void ConfigurableUART::flush(void) noexcept
{
    if (_serial.index >= 0)
    {
        // wait for the buffer to empty
        while ((_serial.tx_head != _serial.tx_tail)) 
        {
            // nop, the interrupt handler will free up space for us
        }
        // and for the hardware to complete sending
        while (serial_tx_active(&_serial)) 
        {
        }
   }
}


void ConfigurableUART::setInterruptPriority(uint32_t priority) noexcept
{
    if (_serial.index >= 0)
    {
        uart_set_interrupt_priority(&_serial, priority);
    }
}

uint32_t ConfigurableUART::getInterruptPriority() noexcept
{
#if 0
// FIXME
    if (_serial.index >= 0)
    {
        return 0;
    }
#endif   
    return 0;
}

bool ConfigurableUART::IsConnected() noexcept
{
    if (_serial.index >= 0) return true;
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
    if  (_serial.index >= 0)
    {
        errs.bufferOverrun = _serial.rx_full;
        errs.framing = _serial.hw_error;
        _serial.hw_error = _serial.rx_full = 0;
    }
    else
    {
        errs.bufferOverrun = 0;
        errs.framing = 0;
    }
	return errs;
}
#endif
