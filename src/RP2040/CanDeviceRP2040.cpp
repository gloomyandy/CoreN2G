/*
 * CanDevice.cpp
 *
 *  Created on: 2 Sep 2020
 *      Author: David
 */

#include "CanDevice.h"

#if SUPPORT_CAN
#if USE_SPICAN
// On devices without a built in CAN-FD module we use an external SPI based device based on the MCP251XFD

// Note that this implementation is very basic and provides just enough functionality to support RRF.

//#include <CoreImp.h>
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
#define MCP_DEBUG 1

static bool core1Initialised = false;
extern "C" void CAN_Handler() noexcept;
extern "C" void Core1Entry() noexcept;

class SPILocker
{
public:
	SPILocker() noexcept { DRV_SPI_Select(); }
	~SPILocker() { DRV_SPI_Deselect();}
	SPILocker(const SPILocker&) = delete;
	SPILocker(SPILocker&&) = delete;
	SPILocker& operator=(const SPILocker&) = delete;
};

static CanDevice devices[NumCanDevices];
constexpr uint32_t AbortTimeout = 100U;


#if MCP_DEBUG
uint32_t bufReqCnt[20];
uint32_t bufFullCnt[20];
#endif

bool CanDevice::ChangeMode(CAN_OPERATION_MODE newMode) noexcept
{
	SPILocker lock;
	DRV_CANFDSPI_OperationModeSelect(0, newMode);
	uint32_t start = millis();
	while(millis() - start < 500)
	{
		if (DRV_CANFDSPI_OperationModeGet(0) == newMode)
		{
			//debugPrintf("Changed mode to %d\n", newMode);
			return true;
		}
		//delay(1);
	}
	debugPrintf("Failed to change to mode %d\n", newMode);
	return false;
}

// Initialise a CAN device and return a pointer to it
/*static*/ CanDevice* CanDevice::Init(unsigned int p_whichCan, unsigned int p_whichPort, const Config& p_config, uint32_t *memStart, const CanTiming &timing, TxEventCallbackFunction p_txCallback) noexcept
{
	int8_t status;
	if (!DRV_SPI_Initialize())
	{
		debugPrintf("Unable to initialise CAN SPI interface, CAN will be disabled\n");
		return nullptr;
	}
	debugPrintf("SPI initialized\n");
	SPILocker lock;
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
#if MCP_DEBUG
	debugPrintf("Configuring rx fifos starting at %d\n", RxBufferNumber::fifo0);
#endif
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

// get bits 2..15 of an address
static inline uint32_t Bits2to15(const volatile void *addr) noexcept
{
	return reinterpret_cast<uint32_t>(addr) & 0x0000FFFC;
}

int otherCore = -1;
// Do the low level hardware initialisation
void CanDevice::DoHardwareInit() noexcept
{
debugPrintf("Init hardware\n");
	run = false;
	abortTx = false;
	latestTimeStamp = 0;
	rxFifos[0].size = config->rxFifo0Size + 1;								// number of entries
	rxFifos[0].buffers = reinterpret_cast<volatile CanRxBuffer*>(rx0Fifo);	// address
	rxFifos[1].size = config->rxFifo1Size + 1;								// number of entries
	rxFifos[1].buffers = reinterpret_cast<volatile CanRxBuffer*>(rx1Fifo);	// address
	txFifo.size = config->txFifoSize + 1;									// number of entries
	txFifo.buffers = reinterpret_cast<volatile CanTxBuffer*>(txBuffers);				// address of transmit fifo - we have no dedicated Tx buffers
debugPrintf("RxSize %d TxSize %d mem size %d rx0 %x rx1 %x tx %x\n", config->GetRxBufferSize(), config->GetTxBufferSize(), config->GetMemorySize(), rx0Fifo, rx1Fifo, txBuffers);
	if (!core1Initialised)
	{
debugPrintf("Starting mcu1 %d %d\n", get_core_num(), otherCore);
delay(100);
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
delay(1000);
debugPrintf("After start %d\n", otherCore);
	// Leave the device disabled. Client must call Enable() to enable it after setting up the receive filters.
}

// Set the extended ID mask. May only be used while the interface is disabled.
void CanDevice::SetExtendedIdMask(uint32_t mask) noexcept
{
}

// Stop and free this device and the CAN port it uses
void CanDevice::DeInit() noexcept
{
}

// Enable this device
void CanDevice::Enable() noexcept
{
	debugPrintf("Enable CAN\n");
	ChangeMode(CAN_NORMAL_MODE);
	run = true;
	debugPrintf("running...\n");
}

// Disable this device
void CanDevice::Disable() noexcept
{
	debugPrintf("Disable CAN\n");
	run = false;
	delay(10);
	ChangeMode(CAN_CONFIGURATION_MODE);
}

static void PrintErrorInfo()
{
#if 0
	uint8_t rec, tec;
	CAN_ERROR_STATE flags;
	DRV_CANFDSPI_ErrorCountStateGet(0, &tec, &rec, &flags);
	CAN_BUS_DIAGNOSTIC bd;
	DRV_CANFDSPI_BusDiagnosticsGet(0, &bd);
	debugPrintf("CAN diag tec %d(%d %d) rec %d(%d %d) dflags %x flags %x mode %d B/O %d\n", tec, bd.bF.errorCount.DTEC, bd.bF.errorCount.NTEC, rec, bd.bF.errorCount.DREC, bd.bF.errorCount.NREC, bd.word[1], flags, DRV_CANFDSPI_OperationModeGet(0), bd.bF.flag.TXBO_ERR);
	DRV_CANFDSPI_BusDiagnosticsClear(0);	
	CAN_MODULE_EVENT eflags;
	DRV_CANFDSPI_ModuleEventGet(0, &eflags);
	CAN_ICODE icode;
	DRV_CANFDSPI_ModuleEventIcodeGet(0, &icode);
	debugPrintf("Event flags %x icode %x\n", eflags, icode);
#endif
}

void CanDevice::CheckBusStatus(uint32_t checkNo) noexcept
{
	// Note SPIMutex must be held when calling this method
	if (DRV_CANFDSPI_OperationModeGet(0) != CAN_NORMAL_MODE)
	{
		// Bus not in operating mode
debugPrintf("Bus not in operating mode %x\n", DRV_CANFDSPI_OperationModeGet(0));
#if 0
		debugPrintf("T:%d Check %d Bus not in correct mode %x\n", millis(), checkNo, DRV_CANFDSPI_OperationModeGet(0));
		PrintErrorInfo();
		CAN_TX_FIFO_STATUS status;
		for(size_t i = 1; i < config->numTxBuffers+2; i++)
		{
			DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
			debugPrintf("chan %d status %x\n", (int)i, status);
		}
#endif
		if (!ChangeMode(CAN_NORMAL_MODE))
		{
			debugPrintf("Change mode 1 failed, try config mode\n");
			ChangeMode(CAN_CONFIGURATION_MODE);
			ChangeMode(CAN_NORMAL_MODE);
			debugPrintf("Bus mode %x\n", DRV_CANFDSPI_OperationModeGet(0));

		}
		CAN_MODULE_EVENT eflags;
		DRV_CANFDSPI_ModuleEventGet(0, &eflags);
		eflags = (CAN_MODULE_EVENT) ((uint32_t) eflags & (int32_t)(CAN_OPERATION_MODE_CHANGE_EVENT|CAN_SYSTEM_ERROR_EVENT|CAN_BUS_ERROR_EVENT|CAN_RX_INVALID_MESSAGE_EVENT));
		if (eflags)
		{

			debugPrintf("Clearing events %x\n", eflags);
			DRV_CANFDSPI_ModuleEventClear(0, eflags);
		}
		PrintErrorInfo();
		if (!busOff)
		{
			debugPrintf("Buss off 1\n");
			busOffCount++;
			busOff = true;
		}
	}
	uint8_t rec, tec;
	CAN_ERROR_STATE flags;
	DRV_CANFDSPI_ErrorCountStateGet(0, &tec, &rec, &flags);
	if ((flags & CAN_TX_BUS_PASSIVE_STATE))
	{
		//debugPrintf("Bus in passive state\n");
#if 0
		debugPrintf("T:%d Check %d tx errors or passive state\n", millis(), checkNo);
		PrintErrorInfo();
		CAN_TX_FIFO_STATUS status;
		for(size_t i = 1; i < config->numTxBuffers+2; i++)
		{
			DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
			debugPrintf("chan %d status %x\n", (int)i, status);
		}
#endif
		CAN_MODULE_EVENT eflags;
		DRV_CANFDSPI_ModuleEventGet(0, &eflags);
		eflags = (CAN_MODULE_EVENT) ((uint32_t) eflags & (int32_t)(CAN_OPERATION_MODE_CHANGE_EVENT|CAN_SYSTEM_ERROR_EVENT|CAN_BUS_ERROR_EVENT|CAN_RX_INVALID_MESSAGE_EVENT));
		if (eflags)
		{

			//debugPrintf("Clearing events %x\n", eflags);
			DRV_CANFDSPI_ModuleEventClear(0, eflags);
		}
		PrintErrorInfo();
		if (!busOff)
		{
			debugPrintf("Bus off 2\n");
			busOffCount++;
			busOff = true;
		}
	}

}

// Drain the Tx event fifo. Can use this instead of supplying a Tx event callback in Init() if we don't expect many events.
void CanDevice::PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept
{
	debugPrintf("Poll events\n");
	SPILocker lock;
	//CheckBusStatus(1);
	CAN_TEF_FIFO_STATUS status;
	DRV_CANFDSPI_TefStatusGet(0, &status);
	//debugPrintf("TEF status %x\n", status);
	while (status & CAN_TEF_FIFO_NOT_EMPTY)
	{
		//debugPrintf("TEF status %x\n", status);
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

uint16_t CanDevice::ReadTimeStampCounter() noexcept
{
	//debugPrintf("read tsc\n");
	//delay(1000);
	latestTimeStamp = 0xffffffff;
	while (latestTimeStamp == 0xffffffff)
	{
	}
	//debugPrintf("end tsc %x\n", latestTimeStamp);
	//delay(1000);
	return latestTimeStamp & 0xffff;
}

uint32_t CanDevice::GetErrorRegister() const noexcept
{
	return 0;
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
	//debugPrintf("check space\n");
	//delay(1000);
	uint32_t start = millis();
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
		bufferFree = nextTxFifoPutIndex != txFifo.getIndex;
		while (!bufferFree && timeout > 0)
		{
			TaskBase::Take(timeout);
			// We may get woken up by other tasks, keep waiting if we need to
			if (timeout != TaskBase::TimeoutUnlimited)
			{
				uint32_t timeSoFar = millis() - start;
				timeout = (timeSoFar >= timeout ? 0 : timeout - timeSoFar);
			}
			bufferFree = nextTxFifoPutIndex != txFifo.getIndex;
		}
		txFifo.waitingTask = nullptr;
		txFifoNotFullInterruptEnabled = false;
	}
#else
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
	//debugPrintf("Send message\n");
	//delay(1000);
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
debugPrintf("Abort message %d %d\n", nextTxFifoPutIndex, txFifo.getIndex);
//delay(100);
		// Cancel transmission of the oldest packet
		abortTx = true;
		do
		{
			delay(1);
		}
		while (nextTxFifoPutIndex == txFifo.getIndex);
debugPrintf("After abort %d %d\n", nextTxFifoPutIndex, txFifo.getIndex);
//delay(1000);
		txBufferFull++;
	}
	{
		SPILocker lock;
		//CheckBusStatus(4);
		//CAN_TX_FIFO_STATUS status;
		//DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
		//debugPrintf("send message buffer %d dst %d typ %d status %x\n", whichBuffer, buffer->id.Dst(), buffer->id.MsgType(), status);
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
		messagesQueuedForSending++;
//debugPrintf("end send\n");
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
	++messagesReceived;
}


// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	const uint32_t start = millis();

//debugPrintf("start rec %d t/o %d\n", whichBuffer, timeout);
//delay(1000);
	if (((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::fifo0) < NumCanRxFifos)
	{
//debugPrintf("start rec2\n");
//delay(1000);
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
			while (getIndex == fifo.putIndex && timeout > 0)
			{
				TaskBase::Take(timeout);
				// We may get woken up by other tasks, keep waiting if we need to
				if (timeout != TaskBase::TimeoutUnlimited)
				{
					uint32_t timeSoFar = millis() - start;
					timeout = (timeSoFar >= timeout ? 0 : timeout - timeSoFar);
				}
			}
			fifo.waitingTask = nullptr;
			if (getIndex == fifo.putIndex)
			{
				debugPrintf("recv timeout %d\n", timeout);
				return false;
			}
		}
#else
		while (getIndex == fifo.putIndex)
		{
			if (millis() - start >= timeout)
			{
				debugPrintf("recv timeout\n");
				return false;
			}
			delay(1);
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
//		debugPrintf("end rec\n");
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
	//debugPrintf("Set EFE index %d limit %d\n", index, config->numExtendedFilterElements);
	if (index < config->numExtendedFilterElements)
	{
		//debugPrintf("Add filter index %d buffer %d id %x mask %x\n", index, whichBuffer, id, mask);
		SPILocker lock;
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

void CanDevice::GetAndClearStats(unsigned int& rMessagesQueuedForSending, unsigned int& rMessagesReceived, unsigned int& rMessagesLost, unsigned int& rBusOffCount) noexcept
{
//#if MCP_DEBUG
#if 0
	{
		SPILocker lock;
		for(size_t i = (size_t)TxBufferNumber::fifo; i <= (size_t)TxBufferNumber::buffer4; i++)
		{
			CAN_TX_FIFO_STATUS status;
			DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
			debugPrintf("chan %d status %x req %d full %d\n", (int)i, status, bufReqCnt[i], bufFullCnt[i]);
			bufFullCnt[i] = bufReqCnt[i] = 0;
		}

		for(size_t i = (size_t)RxBufferNumber::fifo0; i <= (size_t)RxBufferNumber::fifo1; i++)
		{
			CAN_RX_FIFO_STATUS status;
			DRV_CANFDSPI_ReceiveChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
			debugPrintf("chan %d status %x req %d full %d\n", (int)i, status, bufReqCnt[i], bufFullCnt[i]);
			bufFullCnt[i] = bufReqCnt[i] = 0;
		}

		uint8_t rec, tec;
		CAN_ERROR_STATE flags;
		DRV_CANFDSPI_ErrorCountStateGet(0, &tec, &rec, &flags);
		CAN_BUS_DIAGNOSTIC bd;
		DRV_CANFDSPI_BusDiagnosticsGet(0, &bd);
		debugPrintf("CAN diag tec %d rec %d flags %x tx full %d mode %d B/O %d te %d/%d re %d/%d\n", tec, rec, flags, txBufferFull, DRV_CANFDSPI_OperationModeGet(0), bd.bF.flag.TXBO_ERR, bd.bF.errorCount.DTEC, bd.bF.errorCount.NTEC, bd.bF.errorCount.DREC, bd.bF.errorCount.NREC);
		debugPrintf("CRC errors %d rollovers %d\n", crcErrors, rollovers);
		crcErrors = rollovers = 0;
		txBufferFull = 0;
		DRV_CANFDSPI_BusDiagnosticsClear(0);	
	}
#endif
	AtomicCriticalSectionLocker lock;

	rMessagesQueuedForSending = messagesQueuedForSending;
	rMessagesReceived = messagesReceived;
	rMessagesLost = messagesLost;
	rBusOffCount = busOffCount;
	messagesQueuedForSending = messagesReceived = messagesLost = busOffCount = 0;
}

#ifdef RTOS

void CanDevice::Interrupt() noexcept
{
	while ((sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS) != 0)
	{
		const uint32_t ir = sio_hw->fifo_rd;

		// Test whether messages have been received into fifo 0
		constexpr unsigned int rxFifo0WaitingIndex = (unsigned int)RxBufferNumber::fifo0;
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

bool CanDevice::DoReceiveMessage(RxBufferNumber whichBuffer, volatile CanRxBuffer *buffer) noexcept
{
		{
		//SPILocker lock;
		//CheckBusStatus(5);
		CAN_RX_FIFO_STATUS status;
		DRV_CANFDSPI_ReceiveChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
		//debugPrintf("Recv message %d status %x op mode %d\n", whichBuffer, status, DRV_CANFDSPI_OperationModeGet(0));
		if (status & CAN_RX_FIFO_OVERFLOW)
		{
			DRV_CANFDSPI_ReceiveChannelEventOverflowClear(0, (CAN_FIFO_CHANNEL) whichBuffer);
			CAN_RX_FIFO_STATUS status2;
			DRV_CANFDSPI_ReceiveChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status2);
			// We sometimes get a false overflow status, check if we are actually full
			if (status2 & CAN_RX_FIFO_FULL)
				messagesLost++;
#if MCP_DEBUG
			//ebugPrintf("rx queue %d overflow status %x status2 %x\n", whichBuffer, status, status2);
#endif
		}
#if MCP_DEBUG
		if (status & CAN_RX_FIFO_FULL) bufFullCnt[(int)whichBuffer]++;
#endif
		if (status & CAN_RX_FIFO_NOT_EMPTY)
		{
			//debugPrintf("Recv message %d status %x op mode %d\n", whichBuffer, status, DRV_CANFDSPI_OperationModeGet(0));
			CAN_RX_MSGOBJ rxObj;
			DRV_CANFDSPI_ReceiveMessageGet(0, (CAN_FIFO_CHANNEL) whichBuffer, (CAN_RX_MSGOBJ *)&buffer->rxObj, (uint8_t *)buffer->data, sizeof(buffer->data));
			//debugPrintf("src %d dst %d type %d len %d\n", buffer->id.Src(), buffer->id.Dst(), buffer->id.MsgType(), buffer->dataLength);
#if MCP_DEBUG
			bufReqCnt[(int)whichBuffer]++;
#endif
			busOff = false;
			return true;
		}
	}
	return false;
}

bool CanDevice::DoSendMessage(TxBufferNumber whichBuffer, volatile CanTxBuffer *buffer) noexcept
{
	//debugPrintf("Send message\n");
#if MCP_DEBUG
	bufReqCnt[(int)whichBuffer]++;
#endif
	{
		//SPILocker lock;
		//CheckBusStatus(2);
		CAN_TX_FIFO_STATUS status;
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
		//debugPrintf("send message buffer %d dst %d typ %d status %x\n", whichBuffer, buffer->id.Dst(), buffer->id.MsgType(), status);
		if (!(status & CAN_TX_FIFO_NOT_FULL))
		{
#if MCP_DEBUG
			bufFullCnt[(int)whichBuffer]++;
#endif
debugPrintf("Send message no space status %x\n", status);
			CheckBusStatus(2);
			return false;
		}
	}
	{
		//SPILocker lock;
		//CheckBusStatus(4);
		//CAN_TX_FIFO_STATUS status;
		//DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
		//debugPrintf("send message buffer %d dst %d typ %d status %x\n", whichBuffer, buffer->id.Dst(), buffer->id.MsgType(), status);
		uint32_t dlcLen = DLCtoBytes[buffer->txObj.bF.ctrl.DLC];
		DRV_CANFDSPI_TransmitChannelLoad(0, (CAN_FIFO_CHANNEL) whichBuffer, (CAN_TX_MSGOBJ *)&buffer->txObj, (uint8_t *)buffer->data, dlcLen, true);
		messagesQueuedForSending++;
	}
	return true;
}

bool CanDevice::DoAbortMessage(TxBufferNumber whichBuffer) noexcept
{
	debugPrintf("Abort message buffer %d\n", whichBuffer);
	//SPILocker lock;
	CheckBusStatus(3);
	CAN_TX_FIFO_STATUS status;
	uint32_t txReq;
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	debugPrintf("Before cancel txReq %x status %x\n", txReq, status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		//debugPrintf("Space now available, abort not needed\n");
		return true;
	}
#if 0
	// Try disabling retransmission attempts to see if that allows the current message to go....
	DRV_CANFDSPI_TransmitChannelSetTxAttempts(0, (CAN_FIFO_CHANNEL) whichBuffer, 0);
	CheckBusStatus(3);
	txReq = 2;
	DRV_CANFDSPI_TransmitRequestSet(0, (CAN_TXREQ_CHANNEL)txReq);
	uint32_t start = millis();
	do {
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	} while (!(status & CAN_TX_FIFO_NOT_FULL) && (millis() - start) < AbortTimeout);
	// reset the number of attempts to be unlimitted
	DRV_CANFDSPI_TransmitChannelSetTxAttempts(0, (CAN_FIFO_CHANNEL) whichBuffer, 3);
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	debugPrintf("After abort time %d txReq %x status %x\n", millis() - start, txReq, status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		debugPrintf("Request dropped\n");
		return true;
	}
#endif
	DRV_CANFDSPI_TransmitChannelAbort(0, (CAN_FIFO_CHANNEL) whichBuffer);
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	debugPrintf("ater abort req txReq %x status %x\n", txReq, status);
	uint32_t start = millis();
	do {
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	} while (!(status & (CAN_TX_FIFO_NOT_FULL|CAN_TX_FIFO_ABORTED)) && (millis() - start) < AbortTimeout);
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	debugPrintf("After abort time %d txReq %x status %x\n", millis() - start, txReq, status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		debugPrintf("Request aborted\n");
		return true;
	}
	DRV_CANFDSPI_TransmitChannelReset(0, (CAN_FIFO_CHANNEL) whichBuffer);
	do {
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	} while (!(status & CAN_TX_FIFO_NOT_FULL) && (millis() - start) < AbortTimeout);
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	//debugPrintf("After reset time %d txReq %x status %x\n", millis() - start, txReq, status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		debugPrintf("Request reset\n");
		return true;
	}
	debugPrintf("Unable to abort request\n");
	for(size_t i = 1; i < config->numTxBuffers+2; i++)
	{
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
		//debugPrintf("chan %d status %x\n", (int)i, status);
	}
	CheckBusStatus(4);
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		debugPrintf("Request aborted after bus check\n");
		return true;
	}
	debugPrintf("Abort failed\n");
	return false;
}

#if MCP_DEBUG
uint32_t crcErrors = 0;
uint32_t rollovers = 0;
#endif
uint32_t CanDevice::DoReadTimeStampCounter() noexcept
{
	//debugPrintf("get timestamp\n");
	//SPILocker lock;
	uint16_t ts;
	bool goodCrc;
	// The timestamp can sometimes be invalid if it is about to roll over when we read it,
	// or if it has a value that ends in 0x7f or 0x80. We detect these values and read it again
	// See the product errata and othe device drivers.
	do {
		//DRV_CANFDSPI_TimeStampGet(0, &ts);
		DRV_CANFDSPI_ReadByteArrayWithCRC(0, cREGADDR_CiTBC, (uint8_t *)&ts, 2, false, &goodCrc);
#if MCP_DEBUG
	    if (!goodCrc) crcErrors++;
		if (((ts & 0xff) >= 0xf0) ||  ((ts & 0xff) == 0x7f) || ((ts & 0xff) == 0x80)) rollovers++;
#endif
	} while (!goodCrc || ((ts & 0xff) >= 0xf0) ||  ((ts & 0xff) == 0x7f) || ((ts & 0xff) == 0x80));

	return ts & 0xffff;
}
#include <stdio.h>

[[noreturn]] void CanDevice::CanIO() noexcept
{
    //__enable_irq();
	debugPrintf("CanIO running....\n");
	otherCore = get_core_num();
	uint32_t pendingInterrupts = 0;
	for(;;)
	{
		if (run)
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
debugPrintf("After abort\n");
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

#else
#include <Cache.h>
#include <CanSettings.h>
#include <CanMessageBuffer.h>
#include <General/Bitmap.h>
#include "VirtualCanRegisters.h"
#include <cstring>

#undef from
#define PICO_MUTEX_ENABLE_SDK120_COMPATIBILITY	0	// used by mutex.h which is included by multicore.h
#include <pico/multicore.h>
#include <hardware/irq.h>

extern "C" void debugPrintf(const char *fmt, ...) noexcept;

static CanDevice devices[NumCanDevices];

inline uint32_t CanDevice::GetRxBufferSize() const noexcept { return sizeof(CanRxBufferHeader)/sizeof(uint32_t) + (64 >> 2); }
inline uint32_t CanDevice::GetTxBufferSize() const noexcept { return sizeof(CanTxBufferHeader)/sizeof(uint32_t) + (64 >> 2); }
inline CanRxBufferHeader *CanDevice::GetRxFifo0Buffer(uint32_t index) const noexcept { return (CanRxBufferHeader*)(rx0Fifo + (index * GetRxBufferSize())); }
inline CanRxBufferHeader *CanDevice::GetRxFifo1Buffer(uint32_t index) const noexcept { return (CanRxBufferHeader*)(rx1Fifo + (index * GetRxBufferSize())); }
inline CanTxBufferHeader *CanDevice::GetTxBuffer(uint32_t index) const noexcept { return (CanTxBufferHeader*)(txBuffers + (index * GetTxBufferSize())); }

// Virtual registers, shared between the two cores
VirtualCanRegisters virtualRegs;

static bool core1Initialised = false;

extern "C" void CAN_Handler() noexcept;
extern "C" void Core1Entry() noexcept;

// Initialise a CAN device and return a pointer to it
/*static*/ CanDevice* CanDevice::Init(Pin p_txPin, Pin p_rxPin, const Config& p_config, uint32_t *memStart, const CanTiming &timing, TxEventCallbackFunction p_txCallback) noexcept
{
	CanDevice& dev = devices[0];
	if (dev.inUse)														// device instance already in use
	{
		return nullptr;
	}

	// Set up device number, peripheral number, hardware address etc.
	dev.inUse = true;
	dev.config = &p_config;

	// Set up pointers to the individual parts of the buffer memory
	memset(memStart, 0, p_config.GetMemorySize());						// clear out filters, transmit pending flags etc.

	dev.rxStdFilter = (CanStandardMessageFilterElement*)memStart;
	memStart += p_config.GetStandardFiltersMemSize();
	dev.rxExtFilter = (CanExtendedMessageFilterElement*)memStart;
	memStart += p_config.GetExtendedFiltersMemSize();
	dev.rx0Fifo = memStart;
	memStart += (p_config.rxFifo0Size + 1) * p_config.GetRxBufferSize();
	dev.rx1Fifo = memStart;
	memStart += (p_config.rxFifo1Size + 1) * p_config.GetRxBufferSize();
	dev.rxBuffers = memStart;
	memStart += p_config.numRxBuffers * p_config.GetRxBufferSize();
	memStart += p_config.GetTxEventFifoMemSize();
	dev.txBuffers = memStart;

	dev.messagesQueuedForSending = dev.messagesReceived = dev.messagesLost = dev.busOffCount = 0;

	for (unsigned int i = 0; i < p_config.numShortFilterElements; ++i)
	{
		dev.rxStdFilter[i].enabled = false;
	}
	for (unsigned int i = 0; i < p_config.numExtendedFilterElements; ++i)
	{
		dev.rxExtFilter[i].enabled = false;
	}

#ifdef RTOS
	for (volatile TaskHandle& h : dev.txTaskWaiting) { h = nullptr; }
	for (volatile TaskHandle& h : dev.rxTaskWaiting) { h = nullptr; }
	dev.rxBuffersWaiting = 0;
#endif

	virtualRegs.bitrate = 1000000;
	virtualRegs.txPin = p_txPin;
	virtualRegs.rxPin = p_rxPin;

	dev.DoHardwareInit();
	return &dev;
}

// Do the low level hardware initialisation
void CanDevice::DoHardwareInit() noexcept
{
	Disable();			// this also clears some of the virtual registers

	virtualRegs.rxFifos[0].size = config->rxFifo0Size + 1;								// number of entries
	virtualRegs.rxFifos[0].buffers = reinterpret_cast<volatile CanRxBuffer*>(rx0Fifo);	// address
	virtualRegs.rxFifos[1].size = config->rxFifo1Size + 1;								// number of entries
	virtualRegs.rxFifos[1].buffers = reinterpret_cast<volatile CanRxBuffer*>(rx1Fifo);	// address
	virtualRegs.txFifo.size = config->txFifoSize + 1;									// number of entries
	virtualRegs.txFifo.buffers = reinterpret_cast<CanTxBuffer*>(txBuffers);				// address of transmit fifo - we have no dedicated Tx buffers
	virtualRegs.numShortFilterElements = config->numShortFilterElements;				// number of short filter elements
	virtualRegs.shortFiltersAddr = rxStdFilter;											// short filter start address
	virtualRegs.numExtendedFilterElements = config->numExtendedFilterElements;			// number of extended filter elements
	virtualRegs.extendedFiltersAddr = rxExtFilter;										// extended filter start address

	if (!core1Initialised)
	{
		multicore_launch_core1(Core1Entry);
		core1Initialised = true;
	}

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


// Stop and free this device and the CAN port it uses
void CanDevice::DeInit() noexcept
{
	if (inUse)
	{
		Disable();
#ifdef RTOS
		NVIC_DisableIRQ(SIO_IRQ_PROC0_IRQn);
#endif
		inUse = false;														// free the device
	}
}

// Enable this device
void CanDevice::Enable() noexcept
{
	virtualRegs.canEnabled = true;
}

// Disable this device
void CanDevice::Disable() noexcept
{
	virtualRegs.Init();
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif

	bool bufferFree;
	const unsigned int bufferIndex = virtualRegs.txFifo.putIndex;
	unsigned int nextTxFifoPutIndex = bufferIndex + 1;
	if (nextTxFifoPutIndex == virtualRegs.txFifo.size)
	{
		nextTxFifoPutIndex = 0;
	}

#ifdef RTOS
	bufferFree = nextTxFifoPutIndex != virtualRegs.txFifo.getIndex;
	if (!bufferFree && timeout != 0)
	{
		txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();
		virtualRegs.txFifoNotFullInterruptEnabled = true;

		bufferFree = nextTxFifoPutIndex != virtualRegs.txFifo.getIndex;
		// In the following, when we call TaskBase::Take() the Move task sometimes gets woken up early by by the DDA ring
		// Therefore we loop calling Take() until either the call times out or the buffer is free
		while (!bufferFree)
		{
			const bool timedOut = !TaskBase::Take(timeout);
			bufferFree = nextTxFifoPutIndex != virtualRegs.txFifo.getIndex;
			if (timedOut)
			{
				break;
			}
		}
		txTaskWaiting[(unsigned int)whichBuffer] = nullptr;
		virtualRegs.txFifoNotFullInterruptEnabled = false;
	}
#else
	do
	{
		bufferFree = nextTxFifoPutIndex != txFifoGetIndex;
	} while (!bufferFree && millis() - start < timeout);
#endif
	return bufferFree;
}

#if 0	// not currently used

// Return the number of messages waiting to be sent in the transmit FIFO
unsigned int CanDevice::NumTxMessagesPending(TxBufferNumber whichBuffer) noexcept
{
	return READBITS(hw, TXBC, TFQS) - READBITS(hw, TXFQS, TFFL);
}

#endif

void CanDevice::CopyMessageForTransmit(CanMessageBuffer *buffer, volatile CanTxBufferHeader *f) noexcept
{
	if (buffer->extId)
	{
		f->T0.val = buffer->id.GetWholeId();
		f->T0.bit.XTD = 1;
	}
	else
	{
		/* A standard identifier is stored into ID[28:18] */
		f->T0.val = buffer->id.GetWholeId() << 18;
		f->T0.bit.XTD = 0;
	}

	f->T0.bit.RTR = buffer->remote;

	f->T1.bit.MM = buffer->marker;
	f->T1.bit.EFCbit = buffer->reportInFifo;
	uint32_t dataLength = buffer->dataLength;
	volatile uint32_t *dataPtr = ((CanTxBuffer*)f)->data32;
	if (dataLength <= 8)
	{
		f->T1.bit.DLC = dataLength;
		dataPtr[0] = buffer->msg.raw32[0];
		dataPtr[1] = buffer->msg.raw32[1];
	}
	else
	{
		while (dataLength & 3)
		{
			buffer->msg.raw[dataLength++] = 0;				// pad length to a multiple of 4 bytes, setting any additional bytes we send to zero in case the message ends with a string
		}

		if (dataLength <= 24)
		{
			// DLC values 9, 10, 11, 12 code for lengths 12, 16, 20, 24
			uint8_t dlc = (dataLength >> 2) + 6;
			f->T1.bit.DLC = dlc;
			const uint32_t *p = buffer->msg.raw32;
			do
			{
				*dataPtr++ = *p++;
				--dlc;
			} while (dlc != 6);								// copy 3, 4, 5 or 6 words
		}
		else
		{
			// DLC values 13, 14, 15 code for lengths 32, 48, 64
			while (dataLength & 12)
			{
				buffer->msg.raw32[dataLength >> 2] = 0;		// pad length to a multiple of 16 bytes, setting any additional bytes we send to zero in case the message ends with a string
				dataLength += 4;
			}

			uint8_t dlc = (dataLength >> 4) + 11;
			f->T1.bit.DLC = dlc;
			const uint32_t *p = buffer->msg.raw32;
			do
			{
				*dataPtr++ = *p++;
				*dataPtr++ = *p++;
				*dataPtr++ = *p++;
				*dataPtr++ = *p++;
				--dlc;
			} while (dlc != 11);
		}
	}

	f->T1.bit.FDF = buffer->fdMode;
	f->T1.bit.BRS = buffer->useBrs;

	++messagesQueuedForSending;
}

// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
// On return the caller must free or re-use the buffer.
uint32_t CanDevice::SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	uint32_t cancelledId = 0;
	const bool bufferFree = IsSpaceAvailable(whichBuffer, timeout);
	const unsigned int bufferIndex = virtualRegs.txFifo.putIndex;
	unsigned int nextTxFifoPutIndex = bufferIndex + 1;
	if (nextTxFifoPutIndex == virtualRegs.txFifo.size)
	{
		nextTxFifoPutIndex = 0;
	}

	if (!bufferFree)
	{
		// Retrieve details of the packet we are about to cancel
		unsigned int cancelledIndex = nextTxFifoPutIndex + 1;
		if (cancelledIndex == virtualRegs.txFifo.size)
		{
			cancelledIndex = 0;
		}
		cancelledId = GetTxBuffer(cancelledIndex)->T0.bit.ID;

		// Cancel transmission of the oldest packet
		virtualRegs.cancelTransmission = true;
		do
		{
			delay(1);
		}
		while (nextTxFifoPutIndex == virtualRegs.txFifo.getIndex);
	}

	CopyMessageForTransmit(buffer, GetTxBuffer(bufferIndex));
	virtualRegs.txFifo.putIndex = nextTxFifoPutIndex;
	return cancelledId;
}

void CanDevice::CopyReceivedMessage(CanMessageBuffer *null buffer, const volatile CanRxBufferHeader *f) noexcept
{
	// The CAN has written the message directly to memory, so we must invalidate the cache before we read it
	Cache::InvalidateAfterDMAReceive(f, sizeof(CanRxBuffer));					// flush the header data

	if (buffer != nullptr)
	{
		buffer->extId = f->R0.bit.XTD;
		buffer->id.SetReceivedId(f->R0.bit.ID);
		buffer->remote = false;

		const volatile uint32_t *data = f->GetDataPointer();
		buffer->timeStamp = f->R1.bit.RXTS;
		const uint8_t dlc = f->R1.bit.DLC;
		static constexpr uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

		switch (dlc)
		{
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			buffer->msg.raw32[1] = data[1];
			// no break
		case 0:
		case 1:
		case 2:
		case 3:
			buffer->msg.raw32[0] = data[0];
			buffer->dataLength = dlc;
			break;

		case 15:		// 64 bytes
			buffer->msg.raw32[12] = data[12];
			buffer->msg.raw32[13] = data[13];
			buffer->msg.raw32[14] = data[14];
			buffer->msg.raw32[15] = data[15];
			// no break
		case 14:		// 48 bytes
			buffer->msg.raw32[8] = data[8];
			buffer->msg.raw32[9] = data[9];
			buffer->msg.raw32[10] = data[10];
			buffer->msg.raw32[11] = data[11];
			// no break
		case 13:		// 32 bytes
			buffer->msg.raw32[6] = data[6];
			buffer->msg.raw32[7] = data[7];
			// no break
		case 12:		// 24 bytes
			buffer->msg.raw32[5] = data[5];
			// no break
		case 11:		// 20 bytes
			buffer->msg.raw32[4] = data[4];
			// no break
		case 10:		// 16 bytes
			buffer->msg.raw32[3] = data[3];
			// no break
		case 9:			// 12 bytes
			buffer->msg.raw32[0] = data[0];
			buffer->msg.raw32[1] = data[1];
			buffer->msg.raw32[2] = data[2];
			buffer->dataLength = dlc2len[dlc];
		}
//		debugPrintf("Rx typ %u src %u dst %u len %u\n", (unsigned int)buffer->id.MsgType(), buffer->id.Src(), buffer->id.Dst(), buffer->dataLength);
//		delay(100);
	}

	++messagesReceived;
}

// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *null buffer) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif

	if ((unsigned int)whichBuffer < NumCanRxFifos)
	{
		// Check for a received message and wait if necessary
		VirtualCanRegisters::RxFifo& fifo = virtualRegs.rxFifos[(unsigned int)whichBuffer];
		unsigned int getIndex = fifo.getIndex;
#ifdef RTOS
		if (getIndex == fifo.putIndex)
		{
			if (timeout == 0)
			{
				return false;
			}
			TaskBase::ClearCurrentTaskNotifyCount();
			const unsigned int waitingIndex = (unsigned int)whichBuffer;
			rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
			const bool success = (getIndex != fifo.putIndex) || (TaskBase::Take(timeout), getIndex != fifo.putIndex);
			rxTaskWaiting[waitingIndex] = nullptr;
			if (!success)
			{
				return false;
			}
		}
#else
		while (getIndex == fifo.putIndex)
		{
			if (millis() - start >= timeout)
			{
				return false;
			}
		}
#endif
		// Process the received message into the buffer
		CopyReceivedMessage(buffer, &fifo.buffers[fifo.getIndex]);

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
	if ((unsigned int)whichBuffer < NumCanRxFifos)
	{
		// Check for a received message and wait if necessary
		const VirtualCanRegisters::RxFifo& fifo = virtualRegs.rxFifos[(unsigned int)whichBuffer];
		return fifo.getIndex != fifo.putIndex;
	}

	return false;
}

// Disable a short ID filter element
void CanDevice::DisableShortFilterElement(unsigned int index) noexcept
{
	if (index < config->numShortFilterElements)
	{
		rxStdFilter[index].enabled = false;
	}
}

// Set a short ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numShortFilterElements)
	{
		rxStdFilter[index].enabled = false;
		rxStdFilter[index].id = id;
		rxStdFilter[index].mask = mask;
		rxStdFilter[index].whichBuffer = (uint32_t)whichBuffer;
		rxStdFilter[index].enabled = true;
	}
}

// Disable an extended ID filter element
void CanDevice::DisableExtendedFilterElement(unsigned int index) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		rxExtFilter[index].enabled = false;									// disable filter
	}
}

// Set an extended ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		rxExtFilter[index].enabled = false;
		rxExtFilter[index].id = id;
		rxExtFilter[index].mask = mask;
		rxExtFilter[index].whichBuffer = (uint32_t)whichBuffer;
		rxExtFilter[index].enabled = true;
	}
}

void CanDevice::GetLocalCanTiming(CanTiming &timing) noexcept
{
#if 1
	timing.SetDefaults_1Mb();
#else
	const uint32_t localNbtp = hw->REG(NBTP);
	const uint32_t tseg1 = (localNbtp & CAN_(NBTP_NTSEG1_Msk)) >> CAN_(NBTP_NTSEG1_Pos);
	const uint32_t tseg2 = (localNbtp & CAN_(NBTP_NTSEG2_Msk)) >> CAN_(NBTP_NTSEG2_Pos);
	const uint32_t jw = (localNbtp & CAN_(NBTP_NSJW_Msk)) >> CAN_(NBTP_NSJW_Pos);
	const uint32_t brp = (localNbtp & CAN_(NBTP_NBRP_Msk)) >> CAN_(NBTP_NBRP_Pos);
	timing.period = (tseg1 + tseg2 + 3) * (brp + 1);
	timing.tseg1 = (tseg1 + 1) * (brp + 1);
	timing.jumpWidth = (jw + 1) * (brp + 1);
#endif
}

void CanDevice::SetLocalCanTiming(const CanTiming &timing) noexcept
{
#if 0
	UpdateLocalCanTiming(timing);				// set up nbtp and dbtp variables
	Disable();
	hw->REG(NBTP) = nbtp;
	hw->REG(DBTP) = dbtp;
	Enable();
#endif
}

#if 0
void CanDevice::UpdateLocalCanTiming(const CanTiming &timing) noexcept
{
	// Sort out the bit timing
	uint32_t period = timing.period;
	uint32_t tseg1 = timing.tseg1;
	uint32_t jumpWidth = timing.jumpWidth;
	uint32_t prescaler = 1;						// 48MHz main clock
	uint32_t tseg2;

	for (;;)
	{
		tseg2 = period - tseg1 - 1;
		if (tseg1 <= 32 && tseg2 <= 16 && jumpWidth <= 16)
		{
			break;
		}

		// Currently we always use a prescaler that is a power of 2, but we could be more general
		prescaler <<= 1;
		period >>= 1;
		tseg1 >>= 1;
		jumpWidth >>= 1;
	}

#if !SAME70
	bitPeriod = period * prescaler;				// the actual CAN normal bit period, in 48MHz clocks
#endif

	nbtp = ((tseg1 - 1) << CAN_(NBTP_NTSEG1_Pos))
		| ((tseg2 - 1) << CAN_(NBTP_NTSEG2_Pos))
		| ((jumpWidth - 1) << CAN_(NBTP_NSJW_Pos))
		| ((prescaler - 1) << CAN_(NBTP_NBRP_Pos));

	// The fast data rate defaults to the same timing
	dbtp = ((tseg1 - 1) << CAN_(DBTP_DTSEG1_Pos))
		| ((tseg2 - 1) << CAN_(DBTP_DTSEG2_Pos))
		| ((jumpWidth - 1) << CAN_(DBTP_DSJW_Pos))
		| ((prescaler - 1) << CAN_(DBTP_DBRP_Pos));
}
#endif

void CanDevice::GetAndClearStats(unsigned int& rMessagesQueuedForSending, unsigned int& rMessagesReceived, unsigned int& rMessagesLost, unsigned int& rBusOffCount) noexcept
{
	AtomicCriticalSectionLocker lock;

	rMessagesQueuedForSending = messagesQueuedForSending;
	rMessagesReceived = messagesReceived;
	rMessagesLost = messagesLost;
	rBusOffCount = busOffCount;
	messagesQueuedForSending = messagesReceived = messagesLost = busOffCount = 0;
}

void CanDevice::GetAndClearErrorCounts(CanErrorCounts& errs) noexcept
{
	errs = virtualRegs.errors;
	virtualRegs.clearErrorCounts = true;
}

#ifdef RTOS

void CanDevice::Interrupt() noexcept
{
	while ((sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS) != 0)
	{
		const uint32_t ir = sio_hw->fifo_rd;

		// Test whether messages have been received into fifo 0
		constexpr unsigned int rxFifo0WaitingIndex = (unsigned int)RxBufferNumber::fifo0;
		if (ir & VirtualCanRegisters::recdFifo0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo0WaitingIndex]);
		}

		// Test whether messages have been received into fifo 1
		constexpr unsigned int rxFifo1WaitingIndex = (unsigned int)RxBufferNumber::fifo1;
		if (ir & VirtualCanRegisters::recdFifo1)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo1WaitingIndex]);
		}

		// Test whether any messages have been transmitted
		if (ir & VirtualCanRegisters::txFifoNotFull)
		{
			TaskBase::GiveFromISR(txTaskWaiting[(unsigned int)TxBufferNumber::fifo]);
		}
	}
}

// Interrupt handlers

void CAN_Handler() noexcept
{
	devices[0].Interrupt();
}

#endif	// RTOS

#endif
#endif	// SUPPORT_CAN

// End

