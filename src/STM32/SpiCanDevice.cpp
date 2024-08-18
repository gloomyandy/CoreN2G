/*
 * CanDevice.cpp
 *
 *  Created on: 8 Jan 2022
 *  Author: Andy
 */

#include "CanDevice.h"

#if SUPPORT_CAN && !STM32H7
#include <CoreImp.h>
#include <Cache.h>
#include <CanSettings.h>
#include <CanMessageBuffer.h>
#include <General/Bitmap.h>
#include <cstring>
#include <HardwareTimer.h>

// On devices without a built in CAN-FD module we use an external SPI based device based on the MCP251XFD

// Note that this implementation is very basic and provides just enough functionality to support RRF.

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
#include "CanFdSpiApi.h"
#include "CanSpi.h"
#define MCP_DEBUG 0

class SPILocker
{
public:
	SPILocker() noexcept { DRV_SPI_Select(); }
	~SPILocker() { DRV_SPI_Deselect();}
	SPILocker(const SPILocker&) = delete;
	SPILocker(SPILocker&&) = delete;
	SPILocker& operator=(const SPILocker&) = delete;
};

CanDevice CanDevice::devices[NumCanDevices];
constexpr uint32_t AbortTimeout = 100U;

// Clear statistics
void CanDevice::CanStats::Clear() noexcept
{
	messagesQueuedForSending = messagesReceived = messagesLost = protocolErrors = busOffCount = 0;
}

bool CanDevice::ChangeMode(CAN_OPERATION_MODE newMode) noexcept
{
	SPILocker lock;
	DRV_CANFDSPI_OperationModeSelect(0, newMode);
	uint32_t start = millis();
	while(millis() - start < 500)
	{
		if (DRV_CANFDSPI_OperationModeGet(0) == newMode)
		{
			debugPrintf("Changed mode to %d\n", newMode);
			return true;
		}
		delay(1);
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
	SPILocker lock;
	uint32_t resetCnt = 0;
	do {
		if (resetCnt++ > 10)
		{
			debugPrintf("SPI CAN device not in configuration mode\n");
			return nullptr;
		}
		DRV_CANFDSPI_Reset(0);
		delay(100);
		// Hardware should be in configuration mode after a reset
	} while (DRV_CANFDSPI_OperationModeGet(0) != CAN_CONFIGURATION_MODE);
	if (resetCnt > 1)
		debugPrintf("SPI CAN reset complete after %d attempts\n", (int)resetCnt);
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
	config.StoreInTEF = 1;
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

	CAN_TEF_CONFIG tef;
	DRV_CANFDSPI_TefConfigureObjectReset(&tef);
	//tef.FifoSize = p_config.txEventFifoSize;
	tef.FifoSize = 8 - 1;
	tef.TimeStampEnable = 1;
	status = DRV_CANFDSPI_TefConfigure(0, &tef);
	if (status != 0)
	{
		debugPrintf("SPI CAN Failed to set tef config\n");
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
	debugPrintf("Configuring %d tx buffers starting at fifo %d\n", p_config.numTxBuffers, CAN_FIFO_CH2);
#endif
	// We use one tx fifo of size 1 for each "buffer/channel"
	for(size_t i = 0; i < p_config.numTxBuffers; i++)
	{
		CAN_TX_FIFO_CONFIG txc;
		DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txc);
		txc.FifoSize = 1 - 1;
		txc.PayLoadSize = CAN_PLSIZE_64;
		txc.TxAttempts = 3;
		txc.TxPriority = p_config.numTxBuffers - i;
		status = DRV_CANFDSPI_TransmitChannelConfigure(0, (CAN_FIFO_CHANNEL)((size_t)TxBufferNumber::buffer0 + i), &txc);
		if (status != 0)
		{
			debugPrintf("SPI CAN Failed to set txc chan %d config\n", i);
			return nullptr;
		}
	}
#if MCP_DEBUG
	debugPrintf("Configuring rx fifos starting at %d\n", RxBufferNumber::fifo0);
#endif
	// RX fifos
	CAN_RX_FIFO_CONFIG rxc;
	DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxc);
	rxc.FifoSize = 8 - 1;
	rxc.PayLoadSize = CAN_PLSIZE_64;
	rxc.RxTimeStampEnable = 1;
 	status = DRV_CANFDSPI_ReceiveChannelConfigure(0, (CAN_FIFO_CHANNEL)(RxBufferNumber::fifo0), &rxc);
	if (status != 0)
	{
		debugPrintf("SPI CAN Failed to set rxc chan 0 config\n");
		return nullptr;
	}
	rxc.FifoSize = 5 - 1;
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
#if MCP_DEBUG
	uint16_t ts1 = devices[0].ReadTimeStampCounter();
	delay(10);
	debugPrintf("10ms timestamp %u\n", devices[0].ReadTimeStampCounter() - ts1);
#endif

	return &devices[0];
}

// get bits 2..15 of an address
static inline uint32_t Bits2to15(const volatile void *addr) noexcept
{
	return reinterpret_cast<uint32_t>(addr) & 0x0000FFFC;
}

// Do the low level hardware initialisation
void CanDevice::DoHardwareInit() noexcept
{

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
}

// Disable this device
void CanDevice::Disable() noexcept
{
	debugPrintf("Disable CAN\n");
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
			Disable();
			Enable();
		}
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
			stats.busOffCount++;
			busOff = true;
		}
	}
	uint8_t rec, tec;
	CAN_ERROR_STATE flags;
	DRV_CANFDSPI_ErrorCountStateGet(0, &tec, &rec, &flags);
	if ((flags & CAN_TX_BUS_PASSIVE_STATE))
	{
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
			stats.busOffCount++;
			busOff = true;
		}
	}

}

// Drain the Tx event fifo. Can use this instead of supplying a Tx event callback in Init() if we don't expect many events.
void CanDevice::PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept
{
	//debugPrintf("Poll events\n");
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

//#if MCP_DEBUG
uint32_t crcErrors = 0;
uint32_t rollovers = 0;
uint32_t maxTime = 0;
//#endif
void CanDevice::ReadTimeStampCounters(uint16_t& canTimeStamp, uint32_t& stepTimeStamp) noexcept
{
	//debugPrintf("get timestamp\n");
	SPILocker lock;
	uint16_t ts;
	bool goodCrc;
	// The timestamp can sometimes be invalid if it is about to roll over when we read it,
	// or if it has a value that ends in 0x7f or 0x80. We detect these values and read it again
	// See the product errata and othe device drivers.
	for (;;)
	{
		stepTimeStamp = DRV_SPI_GetStepTimerTicks();
		DRV_CANFDSPI_ReadByteArrayWithCRC(0, cREGADDR_CiTBC, (uint8_t *)&ts, 2, false, &goodCrc);
		uint32_t duration = DRV_SPI_GetStepTimerTicks() - stepTimeStamp;
		if (duration > maxTime) maxTime = duration;
		if (!goodCrc) 
			crcErrors++;
		else if (((ts & 0xff) >= 0xf0) ||  ((ts & 0xff) == 0x7f) || ((ts & 0xff) == 0x80)) 
			rollovers++;
		else
			break; 
	}

	canTimeStamp = ts;
}

uint32_t CanDevice::GetErrorRegister() const noexcept
{
	return 0;
}

#if MCP_DEBUG
uint32_t bufReqCnt[20];
uint32_t bufFullCnt[20];
#endif
// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
#if MCP_DEBUG
	bufReqCnt[(int)whichBuffer]++;
#endif
	uint32_t start = millis();
	do
	{
		{
			SPILocker lock;
			//CheckBusStatus(2);
			CAN_TX_FIFO_STATUS status;
			DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
			//debugPrintf("send message buffer %d dst %d typ %d status %x\n", whichBuffer, buffer->id.Dst(), buffer->id.MsgType(), status);
			if (status & CAN_TX_FIFO_NOT_FULL)
				return true;
#if MCP_DEBUG
			bufFullCnt[(int)whichBuffer]++;
#endif
			CheckBusStatus(2);

		}
		delay(1);
	} while (millis() - start <= timeout);

	return false;
}

bool CanDevice::AbortMessage(TxBufferNumber whichBuffer) noexcept
{
	//debugPrintf("Abort message buffer %d\n", whichBuffer);
	SPILocker lock;
	CheckBusStatus(3);
	CAN_TX_FIFO_STATUS status;
	uint32_t txReq;
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	//debugPrintf("Before cancel txReq %x status %x\n", txReq, status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		//debugPrintf("Space now available, abort not needed\n");
		return true;
	}
#if 0
	for(size_t i = 1; i < config->numTxBuffers+2; i++)
	{
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
		debugPrintf("chan %d status %x\n", (int)i, status);
	}
#endif
	DRV_CANFDSPI_TransmitChannelAbort(0, (CAN_FIFO_CHANNEL) whichBuffer);
	uint32_t start = millis();
	do {
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	} while (!(status & (CAN_TX_FIFO_NOT_FULL|CAN_TX_FIFO_ABORTED)) && (millis() - start) < AbortTimeout);
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	//debugPrintf("After abort time %d txReq %x status %x\n", millis() - start, txReq, status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		//debugPrintf("Request aborted\n");
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
		//debugPrintf("Request reset\n");
		return true;
	}
	debugPrintf("Unable to abort request\n");
	for(size_t i = 1; i < config->numTxBuffers+2; i++)
	{
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
		debugPrintf("chan %d status %x\n", (int)i, status);
	}
	CheckBusStatus(4);
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	if (status & CAN_TX_FIFO_NOT_FULL)
	{
		debugPrintf("Request aborted after bus check\n");
		return true;
	}
	return false;
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
	uint32_t ret = 0;
	if (!IsSpaceAvailable(whichBuffer, timeout))
	{
		//debugPrintf("Buffer %d no space\n", whichBuffer);
		if (!AbortMessage(whichBuffer))
		{
			debugPrintf("Abort failed\n");
			txBufferFull++;
			return 0xffffffff;
		}
		ret = 1;
	}
	{
		SPILocker lock;
		//CheckBusStatus(4);
		//CAN_TX_FIFO_STATUS status;
		//DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
		//debugPrintf("send message buffer %d dst %d typ %d status %x\n", whichBuffer, buffer->id.Dst(), buffer->id.MsgType(), status);
		CAN_TX_MSGOBJ txObj;
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

		DRV_CANFDSPI_TransmitChannelLoad(0, (CAN_FIFO_CHANNEL) whichBuffer, &txObj, buffer->msg.raw, dlcLen, true);
		stats.messagesQueuedForSending++;
	}
	return ret;
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
	uint32_t start = millis();
	do
	{
		{
			SPILocker lock;
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
					stats.messagesLost++;
#if MCP_DEBUG
				debugPrintf("rx queue %d overflow status %x status2 %x\n", whichBuffer, status, status2);
#endif
			}
#if MCP_DEBUG
			if (status & CAN_RX_FIFO_FULL) bufFullCnt[(int)whichBuffer]++;
#endif
			if (status & CAN_RX_FIFO_NOT_EMPTY)
			{
				//debugPrintf("Recv message %d status %x op mode %d\n", whichBuffer, status, DRV_CANFDSPI_OperationModeGet(0));
				CAN_RX_MSGOBJ rxObj;
				DRV_CANFDSPI_ReceiveMessageGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &rxObj, buffer->msg.raw, sizeof(buffer->msg.raw));
				CopyHeader(buffer, &rxObj);
				//debugPrintf("src %d dst %d type %d len %d\n", buffer->id.Src(), buffer->id.Dst(), buffer->id.MsgType(), buffer->dataLength);
#if MCP_DEBUG
				bufReqCnt[(int)whichBuffer]++;
#endif
				busOff = false;
				return true;
			}
		}
		delay(1);
	} while (millis() - start <= timeout);


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

void CanDevice::GetAndClearStats(CanDevice::CanStats& dst) noexcept
{
#if MCP_DEBUG
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
		debugPrintf("CRC errors %d rollovers %d max duration %d\n", crcErrors, rollovers, maxTime);
		crcErrors = rollovers = maxTime = 0;
		txBufferFull = 0;
		DRV_CANFDSPI_BusDiagnosticsClear(0);	
	}
#endif
	AtomicCriticalSectionLocker lock;
	dst = stats;
	stats.Clear();
}
#endif