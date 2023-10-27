/*
 * SpiCanRP2040.cpp
 *
 *  Created on: 10 Apr 2023
 *      Author: Andy
 * On devices without a built in CAN-FD module we use an external SPI based device based on the MCP251XFD
 * 
 * On the RP2040 this operation is split into two parts. The main part runs on core 0 and performs higher
 * level operations and initialisation. The lower level functions (which talk to the MCP251XFD during
 * normal operations) run on core 1. 
 */

#include "CanDevice.h"

#if SUPPORT_CAN && USE_SPICAN

#include <Cache.h>
#include <CanSettings.h>
#include <CanMessageBuffer.h>
#include <General/Bitmap.h>
#include <cstring>
#undef from
#define PICO_MUTEX_ENABLE_SDK120_COMPATIBILITY	0	// used by mutex.h which is included by multicore.h
#include <pico/multicore.h>
#include <hardware/irq.h>

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
#include "CanFdSpiApi.h"
#include "CanSpi.h"

static bool core1Initialised = false;
extern "C" void CAN_Handler() noexcept;
extern "C" void Core1Entry() noexcept;

static CanDevice devices[NumCanDevices];
constexpr uint32_t AbortTimeout = 100U;
constexpr uint32_t ChangeModeTimeout = 500U;

bool CanDevice::ChangeMode(CAN_OPERATION_MODE newMode) noexcept
{
	DRV_CANFDSPI_OperationModeSelect(0, newMode);
	uint32_t start = millis();
	while(millis() - start < ChangeModeTimeout)
	{
		if (DRV_CANFDSPI_OperationModeGet(0) == newMode)
		{
			return true;
		}
	}
	debugPrintf("Failed to change to mode %d\n", newMode);
	return false;
}

// Clear statistics
void CanDevice::CanStats::Clear() noexcept
{
	messagesQueuedForSending = messagesReceived = messagesLost = protocolErrors = busOffCount = 0;
}

// Initialise a CAN device and return a pointer to it
/*static*/ CanDevice* CanDevice::Init(unsigned int p_whichCan, unsigned int p_whichPort, const Config& p_config, uint32_t *memStart, const CanTiming &timing, TxEventCallbackFunction p_txCallback) noexcept
{
	int8_t status;
	devices[0].runState = RunState::uninitialised;
	if (!DRV_SPI_Initialize())
	{
		devices[0].runState = RunState::unavailable;
		debugPrintf("Unable to initialise CAN SPI interface, CAN will be disabled\n");
		return nullptr;
	}
	DRV_CANFDSPI_Reset(0);
	// Hardware should be in configuration mode after a reset
	if (DRV_CANFDSPI_OperationModeGet(0) != CAN_CONFIGURATION_MODE)
	{
		debugPrintf("SPI CAN device not in configuration mode\n");
		return nullptr;
	}
	// Configure the clocks
	CAN_OSC_CTRL osc;
	DRV_CANFDSPI_OscillatorControlObjectReset(&osc);
    osc.PllEnable = 0;
    osc.OscDisable = 0;
    osc.SclkDivide = 0;
    osc.ClkOutDivide = 0;
	status = DRV_CANFDSPI_OscillatorControlSet(0, osc);
	if (status != 0)
	{
		debugPrintf("Failed to configure SPI CAN clocks %d\n", status);
		return nullptr;
	}
	CAN_OSC_STATUS ostat;
	status = DRV_CANFDSPI_OscillatorStatusGet(0, &ostat);
	debugPrintf("SPI CAN clock status %d pll %d osc %d sclk %d\n", status, ostat.PllReady, ostat.OscReady, ostat.SclkReady);

	// Now configure can fifos and memory usage
	CAN_CONFIG config;
	DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 0;
    config.TXQEnable = 0;
	config.TxBandWidthSharing = 1;
	config.RestrictReTxAttempts = 1;
	status = DRV_CANFDSPI_Configure(0, &config);
	if (status != 0)
	{
		debugPrintf("SPI CAN Failed to set can config\n");
		return nullptr;
	}
	// TODO use configuration timing values
	status = DRV_CANFDSPI_BitTimeConfigure(0, CAN_1000K_1M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
	if (status != 0)
	{
		debugPrintf("SPI CAN Failed to set bit rates\n");
		return nullptr;
	}

	CAN_TX_FIFO_CONFIG txf;
	DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txf);
	txf.FifoSize = 8 - 1;
	txf.PayLoadSize = CAN_PLSIZE_64;
	txf.TxAttempts = 3;
	txf.TxPriority = 0;
	status = DRV_CANFDSPI_TransmitChannelConfigure(0, (CAN_FIFO_CHANNEL)(TxBufferNumber::fifo), &txf);
	if (status != 0)
	{
		debugPrintf("SPI CAN Failed to set txf chan config\n");
		return nullptr;
	}
	// RX fifos
	CAN_RX_FIFO_CONFIG rxc;
	DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxc);
	rxc.FifoSize = 12 - 1;
	rxc.PayLoadSize = CAN_PLSIZE_64;
    rxc.RxTimeStampEnable = 1;
 	status = DRV_CANFDSPI_ReceiveChannelConfigure(0, (CAN_FIFO_CHANNEL)(RxBufferNumber::fifo0), &rxc);
	if (status != 0)
	{
		debugPrintf("SPI CAN Failed to set rxc chan 0 config\n");
		return nullptr;
	}
	rxc.FifoSize = 1 - 1;
 	status = DRV_CANFDSPI_ReceiveChannelConfigure(0, (CAN_FIFO_CHANNEL)(RxBufferNumber::fifo1), &rxc);
	if (status != 0)
	{
		debugPrintf("SPI CAN Failed to set rxc chan 1 config\n");
		return nullptr;
	}

	status = DRV_CANFDSPI_EccEnable(0);
	if (status != 0)
	{
		debugPrintf("SPI CAN Failed to enable ecc\n");
		return nullptr;
	}
	status = DRV_CANFDSPI_RamInit(0, 0xff);
	if (status != 0)
	{
		debugPrintf("SPI CAN Failed to init ram\n");
		return nullptr;
	}

	devices[0].stats.Clear();

	// Setup timestamps
	devices[0].bitPeriod = 48; // Duet code assumes 48MHz base clock
	DRV_CANFDSPI_TimeStampPrescalerSet(0, 39);
	DRV_CANFDSPI_TimeStampSet(0, 0);
	DRV_CANFDSPI_TimeStampEnable(0);
	status = DRV_CANFDSPI_OscillatorStatusGet(0, &ostat);
	DRV_CANFDSPI_OscillatorEnable(0);
	status = DRV_CANFDSPI_OscillatorStatusGet(0, &ostat);
	devices[0].config = &p_config;
	devices[0].busOff = false;

	memset(memStart, 0, p_config.GetMemorySize()*sizeof(uint32_t));						// clear out filters, transmit pending flags etc.

	devices[0].rx0Fifo = memStart;
	memStart += (p_config.rxFifo0Size + 1) * p_config.GetRxBufferSize();
	devices[0].rx1Fifo = memStart;
	memStart += (p_config.rxFifo1Size + 1) * p_config.GetRxBufferSize();
	devices[0].txBuffers = memStart;
	devices[0].DoHardwareInit();
	return &devices[0];
}

// Do the low level hardware initialisation
void CanDevice::DoHardwareInit() noexcept
{
	runState = RunState::disabled;
	abortTx = false;
	latestTimeStamp = 0;
	rxFifos[0].size = config->rxFifo0Size + 1;								// number of entries
	rxFifos[0].buffers = reinterpret_cast<volatile CanRxBuffer*>(rx0Fifo);	// address
	rxFifos[1].size = config->rxFifo1Size + 1;								// number of entries
	rxFifos[1].buffers = reinterpret_cast<volatile CanRxBuffer*>(rx1Fifo);	// address
	txFifo.size = config->txFifoSize + 1;									// number of entries
	txFifo.buffers = reinterpret_cast<volatile CanTxBuffer*>(txBuffers);				// address of transmit fifo - we have no dedicated Tx buffers
	if (!core1Initialised)
	{
		multicore_reset_core1();
		delay(100);
		multicore_launch_core1(Core1Entry);
		core1Initialised = true;
	}
	else
		debugPrintf("mcu1 already started\n");

	multicore_fifo_drain();

#ifdef RTOS
	const IRQn_Type irqn = SIO_IRQ_PROC0_IRQn;
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);
	irq_set_exclusive_handler(irqn, CAN_Handler);
	NVIC_SetPriority(irqn, 3);
	NVIC_EnableIRQ(irqn);
#endif
	// Leave the device disabled. Client must call Enable() to enable it after setting up the receive filters.
}

// Set the extended ID mask. May only be used while the interface is disabled.
void CanDevice::SetExtendedIdMask(uint32_t mask) noexcept
{
}

// Stop and free this device and the CAN port it uses
void CanDevice::DeInit() noexcept
{
	if (runState != RunState::unavailable)
	{
		Disable();
		runState = RunState::uninitialised;
	}
}

// Enable this device
void CanDevice::Enable() noexcept
{
	if (runState == RunState::disabled)
	{
		ChangeMode(CAN_NORMAL_MODE);
	    // Tell Core 1 it can now use SPI
		runState = RunState::enabled;
	}
}

// Disable this device
void CanDevice::Disable() noexcept
{
	if (runState == RunState::enabled)
	{
		// Disable Core 1 operations
		runState = RunState::disabled;
		delay(10);
		ChangeMode(CAN_CONFIGURATION_MODE);
	}
}

#if 0
// Drain the Tx event fifo. Can use this instead of supplying a Tx event callback in Init() if we don't expect many events.
void CanDevice::PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept
{
	debugPrintf("Poll events\n");
	SPILocker lock;
	CAN_TEF_FIFO_STATUS status;
	DRV_CANFDSPI_TefStatusGet(0, &status);
	while (status & CAN_TEF_FIFO_NOT_EMPTY)
	{
		CanId id;
		CAN_TEF_MSGOBJ tefObj;
		DRV_CANFDSPI_TefMessageGet(0, &tefObj);
		if (tefObj.bF.ctrl.SEQ != 0)
		{
			id.SetReceivedId(tefObj.bF.id.EID | ((uint32_t)tefObj.bF.id.SID << 18));

			p_txCallback(tefObj.bF.ctrl.SEQ, id, tefObj.bF.timeStamp);
			busOff = false;
		}
		DRV_CANFDSPI_TefStatusGet(0, &status);
	}
}
#endif

uint16_t CanDevice::ReadTimeStampCounter() noexcept
{
	latestTimeStamp = 0xffffffff;
	while (latestTimeStamp == 0xffffffff)
	{
	}
	return latestTimeStamp & 0xffff;
}

uint32_t CanDevice::GetErrorRegister() const noexcept
{
	return 0;
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
	const unsigned int bufferIndex = txFifo.putIndex;
	unsigned int nextTxFifoPutIndex = bufferIndex + 1;
	if (nextTxFifoPutIndex == txFifo.size)
	{
		nextTxFifoPutIndex = 0;
	}
	bool bufferFree = nextTxFifoPutIndex != txFifo.getIndex;

#ifdef RTOS
	if (!bufferFree && timeout != 0)
	{
		txFifo.waitingTask = TaskBase::GetCallerTaskHandle();
		txFifoNotFullInterruptEnabled = true;
		// We may get woken up by other tasks, keep waiting if we need to
		while (nextTxFifoPutIndex != txFifo.getIndex)
		{
			if (!TaskBase::Take(timeout))
			{
				break;
			}
		}
		bufferFree = nextTxFifoPutIndex != txFifo.getIndex;
		txFifo.waitingTask = nullptr;
		txFifoNotFullInterruptEnabled = false;
	}
#else
	uint32_t start = millis();
	do
	{
		bufferFree = nextTxFifoPutIndex != txFifoGetIndex;
	} while (!bufferFree && millis() - start < timeout);
#endif
	return bufferFree;

}


static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static const uint8_t BytesToDLC[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 9, 10, 10, 10, 10, 
									 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 
                                     14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
									 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};

// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
// On return the caller must free or re-use the buffer.
uint32_t CanDevice::SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	uint32_t cancelledId = 0;
	const unsigned int bufferIndex = txFifo.putIndex;
	unsigned int nextTxFifoPutIndex = bufferIndex + 1;
	if (nextTxFifoPutIndex == txFifo.size)
	{
		nextTxFifoPutIndex = 0;
	}
	if (!IsSpaceAvailable(whichBuffer, timeout))
	{
		// Retrieve details of the packet we are about to cancel
		unsigned int cancelledIndex = nextTxFifoPutIndex + 1;
		if (cancelledIndex == txFifo.size)
		{
			cancelledIndex = 0;
		}
		cancelledId = txFifo.buffers->txObj.bF.id.EID | (txFifo.buffers->txObj.bF.id.SID << 18);
		// Cancel transmission of the oldest packet
		abortTx = true;
		do
		{
			delay(1);
		}
		while (nextTxFifoPutIndex == txFifo.getIndex);
		txBufferFull++;
	}
	{
		volatile CAN_TX_MSGOBJ& txObj = txFifo.buffers[bufferIndex].txObj;
		txObj.word[0] = 0;
		txObj.word[1] = 0;
		txObj.bF.id.EID = buffer->id.GetWholeId();
		txObj.bF.id.SID = buffer->id.GetWholeId() >> 18;
		txObj.bF.ctrl.IDE = buffer->extId;
		txObj.bF.ctrl.FDF = buffer->fdMode;
		txObj.bF.ctrl.BRS = buffer->useBrs;
		txObj.bF.ctrl.RTR = buffer->remote;
		if (buffer->reportInFifo)
			txObj.bF.ctrl.SEQ = buffer->marker;
		else
			txObj.bF.ctrl.SEQ = 0;
		uint32_t dataLen = buffer->dataLength;
		uint32_t dlcLen = BytesToDLC[dataLen];
		txObj.bF.ctrl.DLC = dlcLen;
		dlcLen = DLCtoBytes[dlcLen];
		while (dataLen < dlcLen)
		{
			buffer->msg.raw[dataLen++] = 0;				// zero fill up to the CANFD buffer length
		}
		memcpy((void *)(txFifo.buffers[bufferIndex].data), buffer->msg.raw, dlcLen);
		txFifo.putIndex = nextTxFifoPutIndex;
		stats.messagesQueuedForSending++;
	}
	return cancelledId;
}

void CanDevice::CopyHeader(CanMessageBuffer *buffer, CAN_RX_MSGOBJ *hdr) noexcept
{
	buffer->extId = hdr->bF.ctrl.IDE;
	buffer->id.SetReceivedId(hdr->bF.id.EID | ((uint32_t)hdr->bF.id.SID << 18));
	buffer->remote = hdr->bF.ctrl.RTR;
	buffer->timeStamp = hdr->bF.timeStamp;
	buffer->dataLength = DLCtoBytes[hdr->bF.ctrl.DLC];
	++stats.messagesReceived;
}


// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	if (((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::fifo0) < NumCanRxFifos)
	{
		// Check for a received message and wait if necessary
		RxFifo& fifo = rxFifos[(uint32_t)whichBuffer - (uint32_t)RxBufferNumber::fifo0];
		unsigned int getIndex = fifo.getIndex;
#if RTOS
		if (getIndex == fifo.putIndex)
		{
			if (timeout == 0)
			{
				return false;
			}
			TaskBase::ClearCurrentTaskNotifyCount();
			fifo.waitingTask = TaskBase::GetCallerTaskHandle();
			// We may get woken up by other tasks, keep waiting if we need to
			while (getIndex == fifo.putIndex)
			{
				if (!TaskBase::Take(timeout))
				{
					break;
				}
			}
			fifo.waitingTask = nullptr;
			if (getIndex == fifo.putIndex)
			{
				return false;
			}
		}
#else
		const uint32_t start = millis();
		while (getIndex == fifo.putIndex)
		{
			if (millis() - start >= timeout)
			{
				return false;
			}
		}
#endif
		// Process the received message into the buffer
		volatile CAN_RX_MSGOBJ& rxObj = fifo.buffers[fifo.getIndex].rxObj;
		memcpy(buffer->msg.raw, (const void *) fifo.buffers[fifo.getIndex].data, DLCtoBytes[rxObj.bF.ctrl.DLC]);
		CopyHeader(buffer, (CAN_RX_MSGOBJ *)&rxObj);
		// Tell the hardware that we have taken the message
		++getIndex;
		if (getIndex == fifo.size)
		{
			getIndex = 0;
		}
		fifo.getIndex = getIndex;
		return true;
	}
	return false;	
}

bool CanDevice::IsMessageAvailable(RxBufferNumber whichBuffer) noexcept
{
	return false;
}

// Disable a short ID filter element
void CanDevice::DisableShortFilterElement(unsigned int index) noexcept
{
}

// Set a short ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{

}

// Disable an extended ID filter element
void CanDevice::DisableExtendedFilterElement(unsigned int index) noexcept
{
}

// Set an extended ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		DRV_CANFDSPI_FilterDisable(0, (CAN_FILTER) index);
		CAN_FILTEROBJ_ID idObj;
		idObj.EID = id;
		idObj.SID = id >> 18;
		idObj.EXIDE = 1;
		mask |= 0x40000000;
		DRV_CANFDSPI_FilterObjectConfigure(0, (CAN_FILTER) index, &idObj);
		CAN_MASKOBJ_ID maskObj;
		maskObj.MEID = mask;
		maskObj.MSID = mask >> 18;
		maskObj.MIDE = 1;
		DRV_CANFDSPI_FilterMaskConfigure(0, (CAN_FILTER) index, &maskObj);
		DRV_CANFDSPI_FilterToFifoLink(0, (CAN_FILTER) index, (CAN_FIFO_CHANNEL) whichBuffer, true);
	}
}

void CanDevice::GetLocalCanTiming(CanTiming &timing) noexcept
{
}

void CanDevice::SetLocalCanTiming(const CanTiming &timing) noexcept
{
}

void CanDevice::UpdateLocalCanTiming(const CanTiming &timing) noexcept
{

}

void CanDevice::GetAndClearStats(CanDevice::CanStats& dst) noexcept
{
	AtomicCriticalSectionLocker lock;
	dst = stats;
	stats.Clear();
}

#ifdef RTOS

void CanDevice::Interrupt() noexcept
{
	while ((sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS) != 0)
	{
		const uint32_t ir = sio_hw->fifo_rd;

		// Test whether messages have been received into fifo 0
		if (ir & rxFifo0NotEmpty)
		{
			TaskBase::GiveFromISR(rxFifos[0].waitingTask);
		}
		// and in fifo1
		if (ir & rxFifo1NotEmpty)
		{
			TaskBase::GiveFromISR(rxFifos[1].waitingTask);
		}

		// Test whether any messages have been transmitted
		if (ir & txFifoNotFull)
		{
			TaskBase::GiveFromISR(txFifo.waitingTask);
		}
	}
}

// Interrupt handlers

void CAN_Handler() noexcept
{
	devices[0].Interrupt();
}

#endif	// RTOS


// The following operations all run on core 1

// To allow flash write operations we need to ensure that we do not execute code from flash
// We place the key functions in RAM and disable CAN during this process
#define CRITICAL_CODE(_name)			__attribute__((section(".time_critical." #_name))) _name
#define CRITICAL_MEMBER(_class, _name)	__attribute__((section(".time_critical." #_class "_" #_name))) _class::_name
#define CRITICAL_DATA_RO(_name)			__attribute__((section(".time_critical." #_name))) _name
#define CRITICAL_DATA_RW(_name)			__attribute__((section(".time_critical." #_name))) _name

void CanDevice::CheckBusStatus() noexcept
{
	// Note SPIMutex must be held when calling this method
	if (DRV_CANFDSPI_OperationModeGet(0) != CAN_NORMAL_MODE)
	{
		// Bus not in operating mode
		if (!ChangeMode(CAN_NORMAL_MODE))
		{
			ChangeMode(CAN_CONFIGURATION_MODE);
			ChangeMode(CAN_NORMAL_MODE);
		}
		CAN_MODULE_EVENT eflags;
		DRV_CANFDSPI_ModuleEventGet(0, &eflags);
		eflags = (CAN_MODULE_EVENT) ((uint32_t) eflags & (int32_t)(CAN_OPERATION_MODE_CHANGE_EVENT|CAN_SYSTEM_ERROR_EVENT|CAN_BUS_ERROR_EVENT|CAN_RX_INVALID_MESSAGE_EVENT));
		if (eflags)
		{
			DRV_CANFDSPI_ModuleEventClear(0, eflags);
		}
		if (!busOff)
		{
			stats.busOffCount++;
			busOff = true;
		}
	}
	uint8_t rec, tec;
	CAN_ERROR_STATE flags;
	DRV_CANFDSPI_ErrorCountStateGet(0, &tec, &rec, &flags);
	if ((flags & CAN_TX_BUS_PASSIVE_STATE))
	{
		CAN_MODULE_EVENT eflags;
		DRV_CANFDSPI_ModuleEventGet(0, &eflags);
		eflags = (CAN_MODULE_EVENT) ((uint32_t) eflags & (int32_t)(CAN_OPERATION_MODE_CHANGE_EVENT|CAN_SYSTEM_ERROR_EVENT|CAN_BUS_ERROR_EVENT|CAN_RX_INVALID_MESSAGE_EVENT));
		if (eflags)
		{
			DRV_CANFDSPI_ModuleEventClear(0, eflags);
		}
		if (!busOff)
		{
			stats.busOffCount++;
			busOff = true;
		}
	}

}


bool CanDevice::DoReceiveMessage(RxBufferNumber whichBuffer, volatile CanRxBuffer *buffer) noexcept
{
	CAN_RX_FIFO_STATUS status;
	// Have we had an overflow of the hardware fifo?
	DRV_CANFDSPI_ReceiveChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	if (status & CAN_RX_FIFO_OVERFLOW)
	{
		DRV_CANFDSPI_ReceiveChannelEventOverflowClear(0, (CAN_FIFO_CHANNEL) whichBuffer);
		CAN_RX_FIFO_STATUS status2;
		DRV_CANFDSPI_ReceiveChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status2);
		// We sometimes get a false overflow status, check if we are actually full
		if (status2 & CAN_RX_FIFO_FULL)
			stats.messagesLost++;
	}
	if (status & CAN_RX_FIFO_NOT_EMPTY)
	{
		DRV_CANFDSPI_ReceiveMessageGet(0, (CAN_FIFO_CHANNEL) whichBuffer, (CAN_RX_MSGOBJ *)&buffer->rxObj, (uint8_t *)buffer->data, sizeof(buffer->data));
		busOff = false;
		return true;
	}
	return false;
}

bool CanDevice::DoSendMessage(TxBufferNumber whichBuffer, volatile CanTxBuffer *buffer) noexcept
{
	CAN_TX_FIFO_STATUS status;
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	if (!(status & CAN_TX_FIFO_NOT_FULL))
	{
		// No space available check to see if the bus has gone offline
		CheckBusStatus();
		return false;
	}

	uint32_t dlcLen = DLCtoBytes[buffer->txObj.bF.ctrl.DLC];
	DRV_CANFDSPI_TransmitChannelLoad(0, (CAN_FIFO_CHANNEL) whichBuffer, (CAN_TX_MSGOBJ *)&buffer->txObj, (uint8_t *)buffer->data, dlcLen, true);
	stats.messagesQueuedForSending++;

	return true;
}

bool CanDevice::DoAbortMessage(TxBufferNumber whichBuffer) noexcept
{
	CheckBusStatus();
	CAN_TX_FIFO_STATUS status;
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		// We now have space so nothing to do
		return true;
	}
	// Stop any ongoing transmission
	DRV_CANFDSPI_TransmitChannelAbort(0, (CAN_FIFO_CHANNEL) whichBuffer);
	uint32_t start = millis();
	do {
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	} while (!(status & (CAN_TX_FIFO_NOT_FULL|CAN_TX_FIFO_ABORTED)) && (millis() - start) < AbortTimeout);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		// We have space
		return true;
	}
	// Clear all of the fifo
	DRV_CANFDSPI_TransmitChannelReset(0, (CAN_FIFO_CHANNEL) whichBuffer);
	do {
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	} while (!(status & CAN_TX_FIFO_NOT_FULL) && (millis() - start) < AbortTimeout);
	// We should now have space
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		return true;
	}
	// This should not happen
	debugPrintf("Unable to abort request\n");
	CheckBusStatus();
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		debugPrintf("Request aborted after bus check\n");
		return true;
	}
	debugPrintf("Abort failed\n");
	return false;
}

uint32_t CanDevice::DoReadTimeStampCounter() noexcept
{
	uint16_t ts;
	bool goodCrc;
	// The timestamp can sometimes be invalid if it is about to roll over when we read it,
	// or if it has a value that ends in 0x7f or 0x80. We detect these values and read it again
	// See the product errata and othe device drivers.
	do {
		DRV_CANFDSPI_ReadByteArrayWithCRC(0, cREGADDR_CiTBC, (uint8_t *)&ts, 2, false, &goodCrc);
	} while (!goodCrc || ((ts & 0xff) >= 0xf0) ||  ((ts & 0xff) == 0x7f) || ((ts & 0xff) == 0x80));

	return ts & 0xffff;
}


[[noreturn]] void CRITICAL_MEMBER(CanDevice, CanIO)() noexcept
{
	debugPrintf("CanIO core 1 running....\n");
	uint32_t pendingInterrupts = 0;
	for(;;)
	{
		if (runState == RunState::enabled)
		{
			// Check for incomming data
			for(size_t rx = 0; rx < NumCanRxFifos; rx++)
			{
				RxFifo& fifo = rxFifos[rx];
				const uint32_t putIndex = fifo.putIndex;
				uint32_t nextPutIndex = putIndex + 1;
				if (nextPutIndex == fifo.size)
				{
					nextPutIndex = 0;
				}
				if (nextPutIndex != fifo.getIndex && DoReceiveMessage((RxBufferNumber)(rx + (uint32_t)RxBufferNumber::fifo0), &fifo.buffers[putIndex]))
				{
					fifo.putIndex = nextPutIndex;
					pendingInterrupts |= (1 << rx);
				}
			}
			if (abortTx)
			{
				DoAbortMessage(TxBufferNumber::fifo);
				abortTx = false;
			}
			{
				TxFifo& fifo = txFifo;
				uint32_t getIndex = fifo.getIndex;
				if (getIndex != fifo.putIndex && DoSendMessage(TxBufferNumber::fifo, &fifo.buffers[getIndex]))
				{
					getIndex = getIndex + 1;
					if (getIndex == fifo.size)
					{
						getIndex = 0;
					}
					fifo.getIndex = getIndex;
					if (txFifoNotFullInterruptEnabled)
					{
						pendingInterrupts |= txFifoNotFull;
					}
				}
			}
			if (latestTimeStamp == 0xffffffff)
			{
				latestTimeStamp = DoReadTimeStampCounter();
			}
			if (pendingInterrupts)
			{
				if ((sio_hw->fifo_st & SIO_FIFO_ST_RDY_BITS) != 0)
				{
						sio_hw->fifo_wr = pendingInterrupts;
						pendingInterrupts = 0;
						__sev();
				}
			}
		}
	}
}

extern "C" [[noreturn]]void Core1Entry() noexcept
{
	devices[0].CanIO();
}

void DisableCanCore1Processing() noexcept
{
	devices[0].Disable();
}

void EnableCanCore1Processing() noexcept
{
	devices[0].Enable();
}

#endif