//Author: sdavi

#ifndef _CONFIGURABLEUART_H_
#define _CONFIGURABLEUART_H_


#include "Core.h"
#include "Stream.h"
#include "Print.h"

#include "uart.h"

// Define config for Serial.begin(baud, config);
// below configs are not supported by STM32
//#define SERIAL_5N1 0x00
//#define SERIAL_5N2 0x08
//#define SERIAL_5E1 0x20
//#define SERIAL_5E2 0x28
//#define SERIAL_5O1 0x30
//#define SERIAL_5O2 0x38
//#define SERIAL_6N1 0x02
//#define SERIAL_6N2 0x0A

#ifdef UART_WORDLENGTH_7B
#define SERIAL_7N1 0x04
#define SERIAL_7N2 0x0C
#define SERIAL_6E1 0x22
#define SERIAL_6E2 0x2A
#define SERIAL_6O1 0x32
#define SERIAL_6O2 0x3A
#endif
#define SERIAL_8N1 0x06
#define SERIAL_8N2 0x0E
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E



class ConfigurableUART : public Stream
{

public:
	typedef void (*InterruptCallbackFn)(ConfigurableUART*) noexcept;
	struct Errors
	{
		uint32_t uartOverrun,
				 framing,
				 bufferOverrun;

		Errors() noexcept : uartOverrun(0), framing(0), bufferOverrun(0)  {  }
	};



    ConfigurableUART() noexcept;

    bool Configure(Pin rx, Pin tx) noexcept;
    
    void begin(uint32_t baud, uint8_t config) noexcept;
    void begin(uint32_t baud) noexcept;
    void end() noexcept;

    size_t write(const uint8_t *buffer, size_t size) noexcept override;
    size_t write(uint8_t c) noexcept override;

    int available(void) noexcept;
    
    int peek(void) noexcept;
    int read(void) noexcept;
    void flush(void) noexcept;
    using Print::write;

    int availableForWrite(void) noexcept;
    size_t canWrite() noexcept;

    bool IsConnected() noexcept;

    int8_t GetUARTPortNumber() noexcept;

    void setInterruptPriority(uint32_t priority) noexcept;
    uint32_t getInterruptPriority() noexcept;

    InterruptCallbackFn SetInterruptCallback(InterruptCallbackFn f) noexcept;

	// Get and clear the errors
	Errors GetAndClearErrors() noexcept;

  protected:
    serial_t _serial;

private:
    size_t writeBlock(const uint8_t *buffer, size_t size) noexcept;
    InterruptCallbackFn interruptCallback;
};

extern ConfigurableUART UART_Slot0;
extern ConfigurableUART UART_Slot1;
extern ConfigurableUART UART_Slot2;


#endif
