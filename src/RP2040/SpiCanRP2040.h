/*
 * CanDevice.h
 *
 *  Created on: 2 Sep 2020
 *      Author: David
 * 		Modified for SpiCan on RP2040 by Andy 10 Apr 2023
 */

#ifndef SRC_SPICANRP2040_H_
#define SRC_SPICANRP2040_H_

#include <CoreIO.h>

#if SUPPORT_CAN && USE_SPICAN 

# include <CanId.h>
# include <General/Bitmap.h>

# ifdef RTOS
#  include <RTOSIface/RTOSIface.h>
# endif

// We only use fifos
constexpr unsigned int MaxTxBuffers = 0;			// maximum number of dedicated transmit buffers supported by this driver
constexpr unsigned int MaxRxBuffers = 0;			// maximum number of dedicated receive buffers supported by this driver

static_assert(MaxTxBuffers <= 31);					// the hardware allows up to 32 if there is no transmit FIFO but our code only supports up to 31 + a FIFO
static_assert(MaxRxBuffers <= 30);					// the hardware allows up to 64 but our code only supports up to 30 + the FIFOs

class CanMessageBuffer;
class CanTiming;
constexpr unsigned int NumCanDevices = 1;			// on other MCUs we only support one CAN device
#include "CanFdSpiDefines.h"
extern "C" [[noreturn]] void Core1Entry() noexcept;

// Queues used to communicate between the two cores.
struct CanRxBuffer
{
	CAN_RX_MSGOBJ rxObj;
	uint8_t data[64];
};

struct CanTxBuffer
{
	CAN_TX_MSGOBJ txObj;
	uint8_t data[64];
};

static constexpr size_t NumCanRxFifos = 2;

struct RxFifo
{
	uint32_t size;									// written be proc 0, during setup only
	volatile CanRxBuffer *buffers;							// written be proc 0, during setup only
	volatile uint32_t getIndex;						// only written by proc 0
	volatile uint32_t putIndex;						// initialised by proc0 then only written by CAN
#if RTOS
	volatile TaskHandle waitingTask;
#endif

	void Clear() noexcept { getIndex = 0; putIndex = 0; }
};

struct TxFifo
{
	uint32_t size;									// written be proc 0, during setup only
	volatile CanTxBuffer *buffers;							// written be proc 0, during setup only
	volatile uint32_t getIndex;						// initialised by proc0 then only written by CAN
	volatile uint32_t putIndex;						// only written by proc 0
#if RTOS
	volatile TaskHandle waitingTask;
#endif

	void Clear() noexcept { getIndex = 0; putIndex = 0; }
};

class CanDevice
{
public:
	enum class RxBufferNumber : uint32_t
	{
		fifo0 = 2, fifo1,
		none = 0xFFFF
	};

	enum class TxBufferNumber : uint32_t
	{
		fifo = 1
	};


	// Struct used to pass configuration constants, with default values
	struct Config
	{
		unsigned int dataSize = 64;											// must be one of: 8, 12, 16, 20, 24, 32, 48, 64
		unsigned int numTxBuffers = 0;
		unsigned int txFifoSize = 4;
		unsigned int numRxBuffers = 0;
		unsigned int rxFifo0Size = 16;
		unsigned int rxFifo1Size = 16;
		unsigned int numShortFilterElements = 0;
		unsigned int numExtendedFilterElements = 3;
		unsigned int txEventFifoSize = 16;

		// Test whether the data size is supported by the CAN hardware
		constexpr bool ValidDataSize() const noexcept
		{
			return dataSize >= 8
				&& (   (dataSize <= 24 && (dataSize & 3) == 0)
					|| (dataSize <= 64 && (dataSize & 15) == 0)
				   );
		}

		// Test whether this is a valid CAN configuration. Use this in a static_assert to check that a specified configuration is valid.
		constexpr bool IsValid() const noexcept
		{
			return ValidDataSize()
				&& numTxBuffers + txFifoSize <= 32							// maximum total Tx buffers supported is 32
				&& numTxBuffers <= MaxTxBuffers								// our code only allows 31 buffers + the FIFO
				&& numRxBuffers <= MaxRxBuffers								// the peripheral supports up to 64 buffers but our code only allows 30 buffers + the two FIFOs
				&& rxFifo0Size <= 64										// max 64 entries per receive FIFO
				&& rxFifo1Size <= 64										// max 64 entries per receive FIFO
				&& txEventFifoSize <= 32;									// max 32 entries in transmit event FIFO
		}

		// Return the number of words of memory occupied by the 11-bit filters
		// We round this up to the next multiple of 8 bytes to reduce the chance of the Tx and Rx buffers crossing cache lines
		constexpr size_t GetStandardFiltersMemSize() const noexcept
		{
			return 0;
		}

		// Return the number of words of memory occupied by the 29-bit filters
		constexpr size_t GetExtendedFiltersMemSize() const noexcept
		{
			return 0;
		}

		// Return the number of words of memory occupied by each transmit buffer
		constexpr size_t GetTxBufferSize() const noexcept
		{
			return (dataSize >> 2) + ((sizeof(CAN_TX_MSGOBJ) + 3) >> 2);
		}

		// Return the number of words of memory occupied by each receive buffer
		constexpr size_t GetRxBufferSize() const noexcept
		{
			return (dataSize >> 2) + ((sizeof(CAN_RX_MSGOBJ) + 3) >> 2);
		}

		// Return the number of words of memory occupied by the transmit event FIFO
		constexpr size_t GetTxEventFifoMemSize() const noexcept
		{
			return 0;
		}

		// Return the total amount of buffer memory needed in 32-bit words. Must be constexpr so we can allocate memory statically in the correct segment.
		constexpr size_t GetMemorySize() const noexcept
		{
			return
				// The RP2040 implementation wastes one slot in each FIFO and has no dedicated buffers
				  (txFifoSize + 1) * GetTxBufferSize()
				+ (rxFifo0Size + rxFifo1Size + 2) * GetRxBufferSize();

		}
	};

	// Type of the callback function called when a transmi event with a nonzero message marker occurs
	typedef void (*TxEventCallbackFunction)(uint8_t marker, CanId id, uint16_t timeStamp) noexcept;

	// Initialise one of the CAN interfaces and return a pointer to the corresponding device. Returns null if device is already in use or device number is out of range.
	static CanDevice *Init(unsigned int p_whichCan, unsigned int p_whichPort, const Config& p_config, uint32_t *memStart, const CanTiming& timing, TxEventCallbackFunction p_txCallback) noexcept;

	// Set the extended ID mask. May only be used while the interface is disabled.
	void SetExtendedIdMask(uint32_t mask) noexcept;

	// Free the device
	void DeInit() noexcept;

	// Enable the device
	void Enable() noexcept;

	// Disable the device
	void Disable() noexcept;

	// Wait for a transmit buffer to become free, with timeout. Return true if it's free.
	bool IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept;

#if 0	// not currently used
	// Return the number of messages waiting to be sent in the transmit FIFO
	unsigned int NumTxMessagesPending(TxBufferNumber whichBuffer) noexcept;
	void PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept;
#endif

	// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
	// Returns the ID of the message that was cancelled, or 0 if we didn't cancel a message.
	uint32_t SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept;

	// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
	bool ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *null buffer) noexcept;

	// Check whether a message is available, returning true if it is
	bool IsMessageAvailable(RxBufferNumber whichBuffer) noexcept;

	// Disable a short ID filter element
	void DisableShortFilterElement(unsigned int index) noexcept;

	// Set a short ID field filter element
	// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
	void SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
		pre(index < NumShortFilterElements);

	// Disable an extended ID filter element
	void DisableExtendedFilterElement(unsigned int index) noexcept;

	// Set an extended ID field filter element
	// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
	void SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
		pre(index < NumShortFilterElements);

	void GetLocalCanTiming(CanTiming& timing) noexcept;

	void SetLocalCanTiming(const CanTiming& timing) noexcept;

	void GetAndClearStats(unsigned int& rMessagesQueuedForSending, unsigned int& rMessagesReceived, unsigned int& rMessagesLost, unsigned int& rBusOffCount) noexcept;

	uint16_t ReadTimeStampCounter() noexcept;

	uint16_t GetTimeStampPeriod() noexcept
	{
		return bitPeriod;
	}

	uint32_t GetErrorRegister() const noexcept;

#ifdef RTOS
	void Interrupt() noexcept;
#endif

	// Configuration constants. Need to be public because they are used to size static data in CanDevice.cpp
	static constexpr size_t Can0DataSize = 64;

private:
	void DoHardwareInit() noexcept;
	void UpdateLocalCanTiming(const CanTiming& timing) noexcept;
	void CopyHeader(CanMessageBuffer *buffer, CAN_RX_MSGOBJ *hdr) noexcept;
	bool ChangeMode(CAN_OPERATION_MODE newMode) noexcept;
	void CheckBusStatus() noexcept;
	[[noreturn]] void CanIO() noexcept;
	bool DoSendMessage(TxBufferNumber whichBuffer, volatile CanTxBuffer *buffer) noexcept;
	bool DoReceiveMessage(RxBufferNumber whichBuffer, volatile CanRxBuffer *buffer) noexcept;
	bool DoAbortMessage(TxBufferNumber whichBuffer) noexcept;
	uint32_t DoReadTimeStampCounter() noexcept;
	
	bool busOff;

	const Config *config;										//!< Configuration parameters
	unsigned int messagesQueuedForSending;
	unsigned int messagesReceived;
	unsigned int messagesLost;									// count of received messages lost because the receive FIFO was full
	unsigned int txBufferFull;									// count of times TX FIFO was full
	unsigned int busOffCount;									// count of the number of times we have reset due to bus off

	uint16_t bitPeriod;											// how many clocks in a CAN normal bit

	bool useFDMode;
	volatile uint32_t *rx0Fifo;									//!< Receive message fifo start
	volatile uint32_t *rx1Fifo;									//!< Receive message fifo start
	volatile uint32_t *txBuffers;								//!< Transmit direct buffers start (the Tx fifo buffers follow them)

	// Following are used to communicate between the two cores
#if RTOS
	static constexpr uint32_t txFifoNotFull = 0x80000000;
	static constexpr uint32_t rxFifo0NotEmpty = 0x1;
	static constexpr uint32_t rxFifo1NotEmpty = 0x2;
#endif
	RxFifo rxFifos[NumCanRxFifos];
	TxFifo txFifo;
	volatile uint32_t latestTimeStamp;
	volatile bool run;											// Process can messages on mcu1
	volatile bool abortTx;										// Abort the current Tx message
	volatile bool txFifoNotFullInterruptEnabled;

	friend void Core1Entry() noexcept;

};

#endif

#endif /* SRC_CANDEVICE_H_ */
