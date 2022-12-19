/*
 * CanDevice.cpp
 *
 *  Created on: 2 Sep 2020
 *      Author: David
 */

#include "CanDevice.h"

#if SUPPORT_CAN
#include <CoreImp.h>
#include <Cache.h>
#include <CanSettings.h>
#include <CanMessageBuffer.h>
#include <General/Bitmap.h>
#include <cstring>
#include <HardwareTimer.h>

#if STM32H7
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static FDCAN_GlobalTypeDef * const CanInstance[2] = {FDCAN1, FDCAN2};
static const IRQn_Type IRQnsByPort[2][2] = { {FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn}, {FDCAN2_IT0_IRQn, FDCAN2_IT1_IRQn} };
static CanDevice *devicesByPort[2] = { nullptr, nullptr };
static Can *hwByPort[2] = {nullptr, nullptr};
CanDevice CanDevice::devices[NumCanDevices];

// Initialise a CAN device and return a pointer to it
/*static*/ CanDevice* CanDevice::Init(unsigned int p_whichCan, unsigned int p_whichPort, const Config& p_config, uint32_t *memStart, const CanTiming &timing, TxEventCallbackFunction p_txCallback) noexcept
{
	if (   p_whichCan >= NumCanDevices									// device number out of range
		|| p_whichPort >= 2												// CAN instance number out of range
		|| devicesByPort[p_whichPort] != nullptr						// CAN instance number already in use
	   )
	{
		return nullptr;
	}

	CanDevice& dev = devices[p_whichCan];
	if (dev.hw.Instance != nullptr)												// device instance already in use
	{
		return nullptr;
	}
	debugPrintf("Init Can device %d port %d\n", p_whichCan, p_whichPort);
	// Set up device number, peripheral number, hardware address etc.
	dev.whichCan = p_whichCan;
	dev.whichPort = p_whichPort;
	dev.config = &p_config;
	dev.txCallback = p_txCallback;
	devicesByPort[p_whichPort] = &dev;
	hwByPort[p_whichPort] = &dev.hw;
	uint32_t dataSize = 0;
	if (p_config.dataSize == 64)
		dataSize = FDCAN_DATA_BYTES_64;
	else if (p_config.dataSize == 8)
		dataSize = FDCAN_DATA_BYTES_8;
	else
	{
		dataSize = FDCAN_DATA_BYTES_64;
		debugPrintf("Unxepected Can data size %d\n", p_config.dataSize);
	}

	dev.hw.Instance = CanInstance[p_whichPort];
	FDCAN_InitTypeDef& Init = dev.hw.Init;
	Init.FrameFormat = (dataSize != FDCAN_DATA_BYTES_8 ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC);
	Init.Mode = FDCAN_MODE_NORMAL;
	Init.AutoRetransmission = ENABLE;
	Init.TransmitPause = ENABLE;
	Init.ProtocolException = ENABLE;
	dev.UpdateLocalCanTiming(timing);
	Init.MessageRAMOffset = 0;
	Init.StdFiltersNbr = p_config.numShortFilterElements;
	Init.ExtFiltersNbr = p_config.numExtendedFilterElements;
	Init.RxFifo0ElmtsNbr = p_config.rxFifo0Size;
	Init.RxFifo0ElmtSize = dataSize;
	Init.RxFifo1ElmtsNbr = p_config.rxFifo1Size;
	Init.RxFifo1ElmtSize = dataSize;
	Init.RxBuffersNbr = p_config.numRxBuffers;
	Init.RxBufferSize = dataSize;
	Init.TxEventsNbr = p_config.txEventFifoSize;
	Init.TxBuffersNbr = p_config.numTxBuffers;
	Init.TxFifoQueueElmtsNbr = p_config.txFifoSize;
	Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	Init.TxElmtSize = dataSize;

	// setup the hardware
	__HAL_RCC_FDCAN_CLK_ENABLE();
  	pinmap_pinout(PB_8, PinMap_CAN_RD);
  	pinmap_pinout(PB_9, PinMap_CAN_TD);

	HAL_StatusTypeDef status = HAL_FDCAN_Init(&dev.hw);
	if (status != HAL_OK)
	{
		debugPrintf("FDCAN init failed %x\n", status);
		return nullptr;
	}
	for(uint32_t i = 0; i < Init.StdFiltersNbr; i++)
		dev.DisableShortFilterElement(i);
	for(uint32_t i = 0; i < Init.ExtFiltersNbr; i++)
		dev.DisableExtendedFilterElement(i);
	status = HAL_FDCAN_ConfigGlobalFilter(&dev.hw, FDCAN_REJECT, FDCAN_REJECT,
                                               		FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	if (status != HAL_OK)
	{
		debugPrintf("Failed to set global filter %x\n", status);
	}
	HAL_FDCAN_ConfigRxFifoOverwrite(&dev.hw, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_BLOCKING);
	HAL_FDCAN_ConfigRxFifoOverwrite(&dev.hw, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_BLOCKING);
	HAL_FDCAN_ConfigFifoWatermark(&dev.hw, FDCAN_CFG_RX_FIFO0, 0);
	HAL_FDCAN_ConfigFifoWatermark(&dev.hw, FDCAN_CFG_RX_FIFO1, 0);
	HAL_FDCAN_ConfigFifoWatermark(&dev.hw, FDCAN_CFG_TX_EVENT_FIFO, 0);
	// Use one CAN bit time as the basis for our timestamps
	status = HAL_FDCAN_ConfigTimestampCounter(&dev.hw, FDCAN_TIMESTAMP_PRESC_1);
	if (status != HAL_OK)
	{
		debugPrintf("FDCAN failed to set t/s prescaler %x\n", status);
	}
	// and enable it using the internal counter
	status = HAL_FDCAN_EnableTimestampCounter(&dev.hw, FDCAN_TIMESTAMP_INTERNAL);
	if (status != HAL_OK)
	{
		debugPrintf("FDCAN failed to enable t/s counter %x\n", status);
	}
	// all interrupts go via int line 0
	status = HAL_FDCAN_ConfigInterruptLines(&dev.hw, FDCAN_IT_TX_COMPLETE|FDCAN_IT_RX_BUFFER_NEW_MESSAGE|FDCAN_IT_TX_EVT_FIFO_NEW_DATA|
													 FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO1_NEW_MESSAGE|FDCAN_IT_BUS_OFF|
													 FDCAN_IT_RX_FIFO0_MESSAGE_LOST|FDCAN_IT_RX_FIFO1_MESSAGE_LOST, 
													 FDCAN_INTERRUPT_LINE0);
	if (status != HAL_OK)
	{
		debugPrintf("FDCAN failed to configure interrupts %x\n", status);
	}
	// Enable the tx complete notification, buffer selection happens later
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_TX_COMPLETE, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_FIFO1_MESSAGE_LOST, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_BUS_OFF, 0);
	HAL_NVIC_EnableIRQ(IRQnsByPort[p_whichPort][0]);
  	HAL_NVIC_EnableIRQ(IRQnsByPort[p_whichPort][1]);

	return &dev;
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
	HAL_StatusTypeDef status = HAL_FDCAN_ConfigExtendedIdMask(&hw, mask);
	if (status != HAL_OK)
		debugPrintf("Failed to set extended ID mask %x\n", status);
}

// Stop and free this device and the CAN port it uses
void CanDevice::DeInit() noexcept
{
	if (hw.Instance != nullptr)
	{
		Disable();
  		HAL_NVIC_DisableIRQ(IRQnsByPort[whichPort][0]);
  		HAL_NVIC_DisableIRQ(IRQnsByPort[whichPort][1]);
		HAL_FDCAN_DeInit(&hw);
		__HAL_RCC_FDCAN_FORCE_RESET();
		__HAL_RCC_FDCAN_RELEASE_RESET();
		devicesByPort[whichPort] = nullptr;									// free the port
		hw.Instance = nullptr;												// free the device
	}
}

// Enable this device
void CanDevice::Enable() noexcept
{
	if (hw.Instance != nullptr)
	{
		HAL_StatusTypeDef status = HAL_FDCAN_Start(&hw);
		if (status != HAL_OK)
			debugPrintf("Failed to enable can device %x\n", status);
	}
	else
		debugPrintf("hw is null\n");
}

// Disable this device
void CanDevice::Disable() noexcept
{
	if (hw.Instance != nullptr)
		HAL_FDCAN_Stop(&hw);
}

// Drain the Tx event fifo. Can use this instead of supplying a Tx event callback in Init() if we don't expect many events.
void CanDevice::PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept
{
	FDCAN_TxEventFifoTypeDef elem;
	while(HAL_FDCAN_GetTxEvent(&hw, &elem) == HAL_OK)
	{
		CanId id;
		id.SetReceivedId(elem.Identifier);
		p_txCallback(elem.MessageMarker, id, elem.TxTimestamp);
	}
}

uint32_t CanDevice::GetErrorRegister() const noexcept
{
	return 0;
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif

	bool bufferFree;
	if (whichBuffer == TxBufferNumber::fifo)
	{
#ifdef RTOS
		bufferFree = HAL_FDCAN_GetTxFifoFreeLevel(&hw) != 0;
		if (!bufferFree && timeout != 0)
		{
			// Get next buffer to be removed
			const unsigned int bufferIndex = ((hw.Instance->TXFQS & FDCAN_TXFQS_TFGI) >> FDCAN_TXFQS_TFGI_Pos);
			const uint32_t trigMask = (uint32_t)1 << bufferIndex;
			txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();

			{
				AtomicCriticalSectionLocker lock;
      			SET_BIT(hw.Instance->TXBTIE, trigMask);
			}

			bufferFree = HAL_FDCAN_GetTxFifoFreeLevel(&hw) != 0;
			// In the following, when we call TaskBase::Take() the Move task sometimes gets woken up early by by the DDA ring
			// Therefore we loop calling Take() until either the call times out or the buffer is free
			while (!bufferFree)
			{
				const bool timedOut = !TaskBase::Take(timeout);
				bufferFree = HAL_FDCAN_GetTxFifoFreeLevel(&hw) != 0;
				if (timedOut)
				{
					break;
				}
			}
			txTaskWaiting[(unsigned int)whichBuffer] = nullptr;

			{
				AtomicCriticalSectionLocker lock;
      			CLEAR_BIT(hw.Instance->TXBTIE, trigMask);
			}
		}
#else
		do
		{
			bufferFree = HAL_FDCAN_GetTxFifoFreeLevel(&hw) != 0;
		} while (!bufferFree && millis() - start < timeout);
#endif
	}
	else
	{
		const unsigned int bufferIndex = (unsigned int)whichBuffer - (unsigned int)TxBufferNumber::buffer0;
		const uint32_t trigMask = (uint32_t)1 << bufferIndex;
#ifdef RTOS
		bufferFree = HAL_FDCAN_IsTxBufferMessagePending(&hw, trigMask) == 0;
		if (!bufferFree && timeout != 0)
		{
			//debugPrintf("Wait space available buffer %d timeout %d\n", whichBuffer, timeout);
			txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();
			{
				AtomicCriticalSectionLocker lock;
      			SET_BIT(hw.Instance->TXBTIE, trigMask);
			}
			bufferFree = HAL_FDCAN_IsTxBufferMessagePending(&hw, trigMask) == 0;

			// In the following, when we call TaskBase::Take() assume that the task may get woken up early
			// Therefore we loop calling Take() until either the call times out or the buffer is free
			while (!bufferFree)
			{
				const bool timedOut = !TaskBase::Take(timeout);
				bufferFree = HAL_FDCAN_IsTxBufferMessagePending(&hw, trigMask) == 0;
				if (timedOut)
				{
					//debugPrintf("Space available timeout\n");
					break;
				}
			}
			txTaskWaiting[(unsigned int)whichBuffer] = nullptr;
			{
				AtomicCriticalSectionLocker lock;
      			CLEAR_BIT(hw.Instance->TXBTIE, trigMask);
			}
		}
#else
		do
		{
			bufferFree = HAL_FDCAN_IsTxBufferMessagePending(&hw, bufferIndex) == 0;
		} while (!bufferFree && millis() - start < timeout);
#endif
	}
	return bufferFree;
}

#if 0	// not currently used

// Return the number of messages waiting to be sent in the transmit FIFO
unsigned int CanDevice::NumTxMessagesPending(TxBufferNumber whichBuffer) noexcept
{
	if (whichBuffer == TxBufferNumber::fifo)
	{
		return READBITS(hw, TXBC, TFQS) - READBITS(hw, TXFQS, TFFL);
	}

	const unsigned int bufferIndex = (unsigned int)whichBuffer - (unsigned int)TxBufferNumber::buffer0;
	return (hw->REG(TXBRP) >> bufferIndex) & 1u;
}

#endif



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
	//debugPrintf("Send message buffer %d num Tx %d\n", whichBuffer, config->numTxBuffers);
	if ((uint32_t)whichBuffer < (uint32_t)TxBufferNumber::buffer0 + config->numTxBuffers)
	{
		const bool bufferFree = IsSpaceAvailable(whichBuffer, timeout);
		const uint32_t bufferIndex = (whichBuffer == TxBufferNumber::fifo)
										? ((hw.Instance->TXFQS & FDCAN_TXFQS_TFGI) >> FDCAN_TXFQS_TFGI_Pos)
											: (uint32_t)whichBuffer - (uint32_t)TxBufferNumber::buffer0;
		const uint32_t trigMask = (uint32_t)1 << bufferIndex;
		if (!bufferFree)
		{
			//debugPrintf("about to cancel message\n");
			// Retrieve details of the packet we are about to cancel
			// FIXME can we get the ID ?
			cancelledId = 1;
			HAL_FDCAN_AbortTxRequest(&hw, trigMask);		
			do
			{
				delay(1);
			}
			while (HAL_FDCAN_IsTxBufferMessagePending(&hw, trigMask) != 0 || (whichBuffer == TxBufferNumber::fifo && HAL_FDCAN_GetTxFifoFreeLevel(&hw) == 0));
			//debugPrintf("Cancel complete\n");
		}
		FDCAN_TxHeaderTypeDef hdr;
		hdr.Identifier = buffer->id.GetWholeId();
		hdr.IdType = (buffer->extId ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID);
		hdr.TxFrameType = (buffer->remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME);
		uint32_t dataLen = buffer->dataLength;
		uint32_t dlcLen = BytesToDLC[dataLen];
		hdr.DataLength = dlcLen << 16;
		dlcLen = DLCtoBytes[dlcLen];
		while (dataLen < dlcLen)
		{
			buffer->msg.raw[dataLen++] = 0;				// zero fill up to the CANFD buffer length
		}
		hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		hdr.BitRateSwitch = (buffer->useBrs ? FDCAN_BRS_ON : FDCAN_BRS_OFF);
		hdr.FDFormat = (buffer->fdMode ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN);
		hdr.TxEventFifoControl = (buffer->reportInFifo ? FDCAN_STORE_TX_EVENTS : FDCAN_NO_TX_EVENTS);
		hdr.MessageMarker = buffer->marker;
		HAL_StatusTypeDef status;
		if (whichBuffer == TxBufferNumber::fifo)
		{
			status = HAL_FDCAN_AddMessageToTxFifoQ(&hw, &hdr, buffer->msg.raw);
			HAL_FDCAN_EnableTxBufferRequest(&hw, HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hw));
		}
		else
		{
			status = HAL_FDCAN_AddMessageToTxBuffer(&hw, &hdr, buffer->msg.raw, trigMask);
			HAL_FDCAN_EnableTxBufferRequest(&hw, trigMask);
		}

		if (status != HAL_OK)
		{
			debugPrintf("Failed to send CAN message %x %x\n", status, (unsigned)hw.ErrorCode);
		}
		else
			++messagesQueuedForSending;
		__DSB();								// this is needed on the SAME70, otherwise incorrect data sometimes gets transmitted
	}
	return cancelledId;
}


void CanDevice::CopyHeader(CanMessageBuffer *buffer, FDCAN_RxHeaderTypeDef *hdr) noexcept
{
	buffer->extId = (hdr->IdType == FDCAN_EXTENDED_ID ? 1 : 0);
	buffer->id.SetReceivedId(hdr->Identifier);
	buffer->remote = (hdr->RxFrameType == FDCAN_REMOTE_FRAME ? 1 : 0);
	buffer->timeStamp = hdr->RxTimestamp;
	buffer->dataLength = DLCtoBytes[hdr->DataLength >> 16];
	++messagesReceived;
}


// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif
	FDCAN_RxHeaderTypeDef hdr;
	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		{
			// Check for a received message and wait if necessary
#ifdef RTOS
			if (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) == 0)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearCurrentTaskNotifyCount();
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) != 0) || (TaskBase::Take(timeout), HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) != 0);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					debugPrintf("Fifo 0 timeout\n");
					return false;
				}
			}
#else
			while (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			//debugPrintf("FIFO0 attempt read\n");
			HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&hw, FDCAN_RX_FIFO0, &hdr, buffer->msg.raw);
			if (status != HAL_OK)
			{
				debugPrintf("Failed to read fifo0 %x\n", status);
			}
			CopyHeader(buffer, &hdr);
		}
		return true;

	case RxBufferNumber::fifo1:
		// Check for a received message and wait if necessary
		{
#ifdef RTOS
			if (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) == 0)
			{
				if (timeout == 0)
				{
					debugPrintf("Fifo 1 timeout\n");
					return false;
				}
				TaskBase::ClearCurrentTaskNotifyCount();
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) != 0) || (TaskBase::Take(timeout), HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) != 0);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			//debugPrintf("FIFO1 attempt read\n");
			HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&hw, FDCAN_RX_FIFO1, &hdr, buffer->msg.raw);
			if (status != HAL_OK)
			{
				debugPrintf("Failed to read fifo1 %x\n", status);
			}
			CopyHeader(buffer, &hdr);
		}
		return true;

	default:
		if ((uint32_t)whichBuffer < (uint32_t)RxBufferNumber::buffer0 + config->numRxBuffers)
		{
			// Check for a received message and wait if necessary
			// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
			const uint32_t bufferNumber = (unsigned int)whichBuffer - (unsigned int)RxBufferNumber::buffer0;
			const uint32_t ndatMask = (uint32_t)1 << bufferNumber;
#ifdef RTOS
			if (HAL_FDCAN_IsRxBufferMessageAvailable(&hw, bufferNumber) == 0)
			{
				if (timeout == 0)
				{
					debugPrintf("Buffer %d timeout\n", (int)bufferNumber);
					return false;
				}
				TaskBase::ClearCurrentTaskNotifyCount();
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				rxBuffersWaiting |= ndatMask;
				const bool success = (HAL_FDCAN_IsRxBufferMessageAvailable(&hw, bufferNumber) != 0 || (TaskBase::Take(timeout), HAL_FDCAN_IsRxBufferMessageAvailable(&hw, bufferNumber) != 0));
				rxBuffersWaiting &= ~ndatMask;
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (HAL_FDCAN_IsRxBufferMessageAvailable(&hw, bufferNumber) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			//debugPrintf("Attempt to read from buffer %d\n", bufferNumber);
			HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&hw, bufferNumber, &hdr, buffer->msg.raw);
			if (status != HAL_OK)
			{
				debugPrintf("Failed to read buffer %x\n", status);
			}
			CopyHeader(buffer, &hdr);
			return true;
		}
		return false;
	}
	delay(100);
	return false;
}

bool CanDevice::IsMessageAvailable(RxBufferNumber whichBuffer) noexcept
{
	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		return HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) != 0;

	case RxBufferNumber::fifo1:
		return HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) != 0;

	default:
		// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
		return HAL_FDCAN_IsRxBufferMessageAvailable(&hw, (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0) != 0;
	}
	return false;
}

// Disable a short ID filter element
void CanDevice::DisableShortFilterElement(unsigned int index) noexcept
{
	if (index < config->numShortFilterElements)
	{
		FDCAN_FilterTypeDef efd;
		efd.IdType = FDCAN_STANDARD_ID;
		efd.FilterIndex = index;
		efd.FilterType = FDCAN_FILTER_MASK;
		efd.FilterID1 = 0;
		efd.IsCalibrationMsg = 0;
		efd.FilterConfig = FDCAN_FILTER_DISABLE;
		efd.FilterID2 = 0;
		HAL_StatusTypeDef status = HAL_FDCAN_ConfigFilter(&hw, &efd);
		if (status != HAL_OK)
		{
			debugPrintf("Failed to disable extended filter %x\n", status);
		}
	}
}

// Set a short ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numShortFilterElements)
	{
		FDCAN_FilterTypeDef efd;
		efd.IdType = FDCAN_STANDARD_ID;
		efd.FilterIndex = index;
		efd.FilterType = FDCAN_FILTER_MASK;
		efd.FilterID1 = id;
		efd.IsCalibrationMsg = 0;
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			efd.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
			efd.FilterID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			efd.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
			efd.FilterID2 = mask;
			break;
		default:
			if ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0 < config->numRxBuffers)
			{
				efd.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
				efd.FilterID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			}
			else
			{
				efd.FilterConfig = FDCAN_FILTER_DISABLE;
				efd.FilterID2 = mask;
			}
			break;
		}
		HAL_StatusTypeDef status = HAL_FDCAN_ConfigFilter(&hw, &efd);
		if (status != HAL_OK)
		{
			debugPrintf("Failed to configure standard filter %x\n", status);
		}
	}
}

// Disable an extended ID filter element
void CanDevice::DisableExtendedFilterElement(unsigned int index) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		FDCAN_FilterTypeDef efd;
		efd.IdType = FDCAN_EXTENDED_ID;
		efd.FilterIndex = index;
		efd.FilterType = FDCAN_FILTER_MASK;
		efd.FilterID1 = 0;
		efd.IsCalibrationMsg = 0;
		efd.FilterConfig = FDCAN_FILTER_DISABLE;
		efd.FilterID2 = 0;
		HAL_StatusTypeDef status = HAL_FDCAN_ConfigFilter(&hw, &efd);
		if (status != HAL_OK)
		{
			debugPrintf("Failed to disable extended filter %x\n", status);
		}

	}
}

// Set an extended ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		//debugPrintf("Add filter index %d buffer %d id %x mask %x\n", index, whichBuffer, id, mask);
		FDCAN_FilterTypeDef efd;
		efd.IdType = FDCAN_EXTENDED_ID;
		efd.FilterIndex = index;
		efd.FilterType = FDCAN_FILTER_MASK;
		efd.FilterID1 = id;
		efd.IsCalibrationMsg = 0;
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			efd.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
			efd.FilterID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			efd.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
			efd.FilterID2 = mask;
			break;
		default:
			if ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0 < config->numRxBuffers)
			{
				efd.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
				efd.FilterID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			}
			else
			{
				efd.FilterConfig = FDCAN_FILTER_DISABLE;
				efd.FilterID2 = mask;
			}
			break;
		}
		HAL_StatusTypeDef status = HAL_FDCAN_ConfigFilter(&hw, &efd);
		if (status != HAL_OK)
		{
			debugPrintf("Failed to configure extended filter %x\n", status);
		}
	}
}

void CanDevice::GetLocalCanTiming(CanTiming &timing) noexcept
{

	const uint32_t localNbtp = hw.Instance->NBTP;
	const uint32_t tseg1 = (localNbtp & FDCAN_NBTP_NTSEG1_Msk) >> FDCAN_NBTP_NTSEG1_Pos;
	const uint32_t tseg2 = (localNbtp & FDCAN_NBTP_NTSEG2_Msk) >> FDCAN_NBTP_NTSEG2_Pos;
	const uint32_t jw = (localNbtp & FDCAN_NBTP_NSJW_Msk) >> FDCAN_NBTP_NSJW_Pos;
	const uint32_t brp = (localNbtp & FDCAN_NBTP_NBRP_Msk) >> FDCAN_NBTP_NBRP_Pos;
	timing.period = (tseg1 + tseg2 + 3) * (brp + 1);
	timing.tseg1 = (tseg1 + 1) * (brp + 1);
	timing.jumpWidth = (jw + 1) * (brp + 1);
}

void CanDevice::SetLocalCanTiming(const CanTiming &timing) noexcept
{
	UpdateLocalCanTiming(timing);				// set up nbtp and dbtp variables
	Disable();
  	hw.Instance->NBTP = ((((uint32_t)hw.Init.NominalSyncJumpWidth - 1U) << FDCAN_NBTP_NSJW_Pos) |
                            (((uint32_t)hw.Init.NominalTimeSeg1 - 1U) << FDCAN_NBTP_NTSEG1_Pos)    |
                            (((uint32_t)hw.Init.NominalTimeSeg2 - 1U) << FDCAN_NBTP_NTSEG2_Pos)    |
                            (((uint32_t)hw.Init.NominalPrescaler - 1U) << FDCAN_NBTP_NBRP_Pos));

    hw.Instance->DBTP = ((((uint32_t)hw.Init.DataSyncJumpWidth - 1U) << FDCAN_DBTP_DSJW_Pos) |
                              (((uint32_t)hw.Init.DataTimeSeg1 - 1U) << FDCAN_DBTP_DTSEG1_Pos)    |
                              (((uint32_t)hw.Init.DataTimeSeg2 - 1U) << FDCAN_DBTP_DTSEG2_Pos)    |
                              (((uint32_t)hw.Init.DataPrescaler - 1U) << FDCAN_DBTP_DBRP_Pos));
	Enable();
}

void CanDevice::UpdateLocalCanTiming(const CanTiming &timing) noexcept
{
	// Sort out the bit timing
	uint32_t period = timing.period;
	uint32_t tseg1 = timing.tseg1;
	uint32_t jumpWidth = timing.jumpWidth;
	uint32_t prescaler = 1;						// 48MHz main clock
	uint32_t tseg2 = period - tseg1 - 1;
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
	FDCAN_InitTypeDef& Init = hw.Init;
	Init.NominalPrescaler = prescaler; /* tq = NominalPrescaler x (1/fdcan_ker_ck) */
	Init.NominalSyncJumpWidth = jumpWidth;
  	Init.NominalTimeSeg1 = tseg1; /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 */
  	Init.NominalTimeSeg2 = tseg2;
	Init.DataPrescaler = prescaler;
	Init.DataSyncJumpWidth = jumpWidth;
	Init.DataTimeSeg1 = tseg1; /* DataTimeSeg1 = Propagation_segment + Phase_segment_1 */
	Init.DataTimeSeg2 = tseg2;
}

void CanDevice::GetAndClearStats(unsigned int& rMessagesQueuedForSending, unsigned int& rMessagesReceived, unsigned int& rMessagesLost, unsigned int& rBusOffCount) noexcept
{
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
#if 0
	uint32_t ir;
	while ((ir = hw->REG(IR) & statusMask) != 0)
	{
		hw->REG(IR) = ir;

		constexpr unsigned int rxFifo0WaitingIndex = (unsigned int)RxBufferNumber::fifo0;
		if ((ir & CAN_(IR_RF0N)) != 0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo0WaitingIndex]);
		}

		constexpr unsigned int rxFifo1WaitingIndex = (unsigned int)RxBufferNumber::fifo1;
		if ((ir & CAN_(IR_RF1N)) != 0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo1WaitingIndex]);
		}

		if (ir & CAN_(IR_DRX))
		{
			// Check which receive buffers have new messages
			uint32_t newData;
			while (((newData = hw->REG(NDAT1)) & rxBuffersWaiting) != 0)
			{
				const unsigned int rxBufferNumber = LowestSetBit(newData);
				rxBuffersWaiting &= ~((uint32_t)1 << rxBufferNumber);
				const unsigned int waitingIndex = rxBufferNumber + (unsigned int)RxBufferNumber::buffer0;
				if (waitingIndex < ARRAY_SIZE(rxTaskWaiting))
				{
					TaskBase::GiveFromISR(rxTaskWaiting[waitingIndex]);
				}
			}
		}

		if (ir & CAN_(IR_TC))
		{
			// Check which transmit buffers have finished transmitting
			uint32_t transmitDone;
			while ((transmitDone = (~hw->REG(TXBRP)) & hw->REG(TXBTIE)) != 0)
			{
				const unsigned int bufferNumber = LowestSetBit(transmitDone);
				hw->REG(TXBTIE) &= ~((uint32_t)1 << bufferNumber);
				if (bufferNumber < READBITS(hw, TXBC, NDTB))
				{
					// Completed transmission from a dedicated transmit buffer
					const unsigned int waitingIndex = bufferNumber + (unsigned int)TxBufferNumber::buffer0;
					if (waitingIndex < ARRAY_SIZE(txTaskWaiting))
					{
						TaskBase::GiveFromISR(txTaskWaiting[waitingIndex]);
					}
				}
				else
				{
					// Completed transmission from a transmit FIFO buffer
					TaskBase::GiveFromISR(txTaskWaiting[(unsigned int)TxBufferNumber::fifo]);
				}
			}
		}

		if (ir & CAN_(IR_BO))
		{
			++busOffCount;
			DoHardwareInit();
			Enable();
			return;
		}

		if (ir & (CAN_(IR_RF0L) | CAN_(IR_RF1L)))
		{
			++messagesLost;
		}

		if (ir & CAN_(IR_TEFN))
		{
			uint32_t txefs;
			while (((txefs = hw->REG(TXEFS)) & CAN_(TXEFS_EFFL_Msk)) != 0)
			{
				const uint32_t index = (txefs & CAN_(TXEFS_EFGI_Msk)) >> CAN_(TXEFS_EFGI_Pos);
				const TxEvent* elem = GetTxEvent(index);
				if (elem->R1.bit.ET == 1)
				{
					CanId id;
					id.SetReceivedId(elem->R0.bit.ID);
					txCallback(elem->R1.bit.MM, id, elem->R1.bit.TXTS);
				}
				hw->REG(TXEFA) = index;
			}
		}
	}
#endif
}

// Interrupt handlers
extern "C" void FDCAN1_IT0_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[0]);
}

extern "C" void FDCAN2_IT0_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[1]);
}

extern "C" void FDCAN1_IT1_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[0]);
}

extern "C" void FDCAN2_IT1_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[1]);
}

extern "C" void FDCAN_CAL_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[0]);
}

extern "C" void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
	if ((TxEventFifoITs & FDCAN_IT_TX_EVT_FIFO_NEW_DATA) != 0)
	{
		FDCAN_TxEventFifoTypeDef elem;
		while(HAL_FDCAN_GetTxEvent(hfdcan, &elem) == HAL_OK)
		{
			CanId id;
			id.SetReceivedId(elem.Identifier);
			if (hfdcan == hwByPort[0])
				devicesByPort[0]->txCallback(elem.MessageMarker, id, elem.TxTimestamp);
			else
				devicesByPort[1]->txCallback(elem.MessageMarker, id, elem.TxTimestamp);
		}
	}

}

extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t transmitDone)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	while (transmitDone != 0)
	{
		const unsigned int bufferNumber = LowestSetBit(transmitDone);
		transmitDone &= ~((uint32_t)1 << bufferNumber);
		if (bufferNumber <= dev->hw.Init.TxBuffersNbr )
		{
			// Completed transmission from a dedicated transmit buffer
			const unsigned int waitingIndex = bufferNumber + (unsigned int)CanDevice::TxBufferNumber::buffer0;
			if (waitingIndex < ARRAY_SIZE(dev->txTaskWaiting))
			{
				TaskBase::GiveFromISR(dev->txTaskWaiting[waitingIndex]);
			}
		}
		else
		{
			// Completed transmission from a transmit FIFO buffer
			TaskBase::GiveFromISR(dev->txTaskWaiting[(unsigned int)CanDevice::TxBufferNumber::fifo]);
		}
	}
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
		TaskBase::GiveFromISR(dev->rxTaskWaiting[(unsigned int)CanDevice::RxBufferNumber::fifo0]);
	}
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0)
	{
		dev->messagesLost++;
	}
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0)
	{
		TaskBase::GiveFromISR(dev->rxTaskWaiting[(unsigned int)CanDevice::RxBufferNumber::fifo1]);
	}
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0)
	{
		dev->messagesLost++;
	}
}

extern "C" void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	uint32_t newData;
	while (((newData = dev->hw.Instance->NDAT1) & dev->rxBuffersWaiting) != 0)
	{
		const unsigned int rxBufferNumber = LowestSetBit(newData);
		dev->rxBuffersWaiting &= ~((uint32_t)1 << rxBufferNumber);
		const unsigned int waitingIndex = rxBufferNumber + (unsigned int)CanDevice::RxBufferNumber::buffer0;
		if (waitingIndex < ARRAY_SIZE(dev->rxTaskWaiting))
		{
			TaskBase::GiveFromISR(dev->rxTaskWaiting[waitingIndex]);
		}
	}
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0)
	{
		dev->busOffCount++;
		dev->Disable();
		dev->Enable();
	}
}

#endif
#else
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
#include "drv_canfdspi_api.h"
#include "drv_spi.h"
CanDevice CanDevice::devices[NumCanDevices];
static Mutex SPIMutex;

bool CanDevice::ChangeMode(CAN_OPERATION_MODE newMode) noexcept
{
	MutexLocker lock(SPIMutex);
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
	debugPrintf("Can Init\n");
	DRV_SPI_Initialize();
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
    osc.SclkDivide = 1;
    osc.ClkOutDivide = 1;
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

	status = DRV_CANFDSPI_BitTimeConfigure(0, CAN_1000K_1M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_20M);
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
	debugPrintf("Configuring %d tx buffers starting at fifo %d\n", p_config.numTxBuffers, CAN_FIFO_CH2);
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
	debugPrintf("Configuring rx fifos starting at %d\n", RxBufferNumber::fifo0);
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

	// Setup timestamps
	devices[0].bitPeriod = 48; // Duet code assumes 48MHz base clock
	DRV_CANFDSPI_TimeStampPrescalerSet(0, 19);
	DRV_CANFDSPI_TimeStampSet(0, 0);
	DRV_CANFDSPI_TimeStampEnable(0);
	status = DRV_CANFDSPI_OscillatorStatusGet(0, &ostat);
	debugPrintf("SPI CAN clock status %d pll %d osc %d sclk %d\n", status, ostat.PllReady, ostat.OscReady, ostat.SclkReady);
	DRV_CANFDSPI_OscillatorEnable(0);
	status = DRV_CANFDSPI_OscillatorStatusGet(0, &ostat);
	debugPrintf("SPI CAN clock status %d pll %d osc %d sclk %d\n", status, ostat.PllReady, ostat.OscReady, ostat.SclkReady);
	devices[0].config = &p_config;
	SPIMutex.Create("CanTrans");
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
}

void CanDevice::CheckBusStatus(uint32_t checkNo) noexcept
{
	// Note SPIMutex must be held when calling this method
	if (DRV_CANFDSPI_OperationModeGet(0) != CAN_NORMAL_MODE)
	{
		// Bus not in operating mode
		debugPrintf("T:%d Check %d Bus not in correct mode %x\n", millis(), checkNo, DRV_CANFDSPI_OperationModeGet(0));
		PrintErrorInfo();
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

			debugPrintf("Clearing events %x\n", eflags);
			DRV_CANFDSPI_ModuleEventClear(0, eflags);
		}
		PrintErrorInfo();
		busOffCount++;
	}
	uint8_t rec, tec;
	CAN_ERROR_STATE flags;
	DRV_CANFDSPI_ErrorCountStateGet(0, &tec, &rec, &flags);
	if ((flags & CAN_TX_BUS_PASSIVE_STATE))
	{
		debugPrintf("T:%d Check %d tx errors or passive state\n", millis(), checkNo);
		PrintErrorInfo();
		CAN_TX_FIFO_STATUS status;
		for(size_t i = 1; i < config->numTxBuffers+2; i++)
		{
			DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
			debugPrintf("chan %d status %x\n", (int)i, status);
		}
		//Disable();
		//Enable();
		CAN_MODULE_EVENT eflags;
		DRV_CANFDSPI_ModuleEventGet(0, &eflags);
		eflags = (CAN_MODULE_EVENT) ((uint32_t) eflags & (int32_t)(CAN_OPERATION_MODE_CHANGE_EVENT|CAN_SYSTEM_ERROR_EVENT|CAN_BUS_ERROR_EVENT|CAN_RX_INVALID_MESSAGE_EVENT));
		if (eflags)
		{

			debugPrintf("Clearing events %x\n", eflags);
			DRV_CANFDSPI_ModuleEventClear(0, eflags);
		}
		PrintErrorInfo();
		busOffCount++;
	}

}

// Drain the Tx event fifo. Can use this instead of supplying a Tx event callback in Init() if we don't expect many events.
void CanDevice::PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept
{
	//debugPrintf("Poll events\n");
	MutexLocker lock(SPIMutex);
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
		}
		DRV_CANFDSPI_TefStatusGet(0, &status);
	}
}

uint32_t crcErrors = 0;
uint32_t rollovers = 0;

uint16_t CanDevice::ReadTimeStampCounter() noexcept
{
	//debugPrintf("get timestamp\n");
	MutexLocker lock(SPIMutex);
	uint16_t ts;
	int8_t status;
	bool goodCrc;
	do {
		//DRV_CANFDSPI_TimeStampGet(0, &ts);
		status = DRV_CANFDSPI_ReadByteArrayWithCRC(0, cREGADDR_CiTBC,
			(uint8_t *)&ts, 2, false, &goodCrc);
    
	    if (!goodCrc) crcErrors++;
		if ((ts & 0xff) >= 0xfe) rollovers++;
	} while (!goodCrc || (ts & 0xff) >= 0xfe);

	return ts & 0xffff;
}

uint32_t CanDevice::GetErrorRegister() const noexcept
{
	return 0;
}

uint32_t bufReqCnt[20];
uint32_t bufFullCnt[20];

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
	bufReqCnt[(int)whichBuffer]++;
	uint32_t start = millis();
	do
	{
		{
			MutexLocker lock(SPIMutex);
			//CheckBusStatus(2);
			CAN_TX_FIFO_STATUS status;
			DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
			//debugPrintf("send message buffer %d dst %d typ %d status %x\n", whichBuffer, buffer->id.Dst(), buffer->id.MsgType(), status);
			if (status & CAN_TX_FIFO_NOT_FULL)
				return true;
			bufFullCnt[(int)whichBuffer]++;
			CheckBusStatus(2);

		}
		//delay(1);
	} while (millis() - start <= timeout);

	return false;
}

void CanDevice::AbortMessage(TxBufferNumber whichBuffer) noexcept
{
	debugPrintf("Abort message buffer %d\n", whichBuffer);
	MutexLocker lock(SPIMutex);
	CheckBusStatus(3);
#if 0
	uint8_t rec, tec;
	CAN_ERROR_STATE flags;
	DRV_CANFDSPI_ErrorCountStateGet(0, &tec, &rec, &flags);
	CAN_BUS_DIAGNOSTIC bd;
	DRV_CANFDSPI_BusDiagnosticsGet(0, &bd);
	debugPrintf("CAN diag tec %d rec %d flags %x tx full %d mode %d B/O %d\n", tec, rec, flags, txBufferFull, DRV_CANFDSPI_OperationModeGet(0), bd.bF.flag.TXBO_ERR);
	CAN_TX_FIFO_STATUS status;
	for(size_t i = 1; i < config->numTxBuffers+2; i++)
	{
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
		debugPrintf("chan %d status %x\n", (int)i, status);
	}
	if (tec > 127 || (flags & CAN_TX_BUS_PASSIVE_STATE) || DRV_CANFDSPI_OperationModeGet(0) != CAN_NORMAL_MODE)
	{
		debugPrintf("Reset all tx channels\n");
		Disable();
		Enable();
#if 0
		DRV_CANFDSPI_TransmitAbortAll(0);
		delay(100);
		for(size_t i = 1; i < config->numTxBuffers+2; i++)
		{
			DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
			debugPrintf("chan %d status %x\n", (int)i, status);

			DRV_CANFDSPI_TransmitChannelReset(0, (CAN_FIFO_CHANNEL) i);
			delay(10);
			DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) i, &status);
			debugPrintf("chan %d status %x\n", (int)i, status);
		}
#endif
	}
#endif
	CAN_TX_FIFO_STATUS status;
	uint32_t txReq;
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	debugPrintf("Before cancel txReq %x status %x\n", txReq, status);
	DRV_CANFDSPI_TransmitChannelAbort(0, (CAN_FIFO_CHANNEL) whichBuffer);
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	debugPrintf("After1 cancel txReq %x status %x\n", txReq, status);
	delay(100);
	DRV_CANFDSPI_TransmitChannelReset(0, (CAN_FIFO_CHANNEL) whichBuffer);
	DRV_CANFDSPI_TransmitRequestGet(0, &txReq);
	DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
	debugPrintf("After2 cancel txReq %x status %x\n", txReq, status);
	for(size_t i = 1; i < config->numTxBuffers+2; i++)
	{
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
		debugPrintf("chan %d status %x\n", (int)i, status);
	}

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
	if (!IsSpaceAvailable(whichBuffer, timeout))
	{
		debugPrintf("Buffer %d no space\n", whichBuffer);
		AbortMessage(whichBuffer);
	}
	{
		MutexLocker lock(SPIMutex);
		//CheckBusStatus(4);
		CAN_TX_FIFO_STATUS status;
		DRV_CANFDSPI_TransmitChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
		//debugPrintf("send message buffer %d dst %d typ %d status %x\n", whichBuffer, buffer->id.Dst(), buffer->id.MsgType(), status);
		if (status & CAN_TX_FIFO_NOT_FULL)
		{
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
			messagesQueuedForSending++;
		}
		else
		{
			// FIXME handle no space to send
			debugPrintf("No space after abort\n");
			txBufferFull++;
		}
	}
	return 0;
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
	uint32_t start = millis();
	do
	{
		{
			MutexLocker lock(SPIMutex);
			//CheckBusStatus(5);
			CAN_RX_FIFO_STATUS status;
			DRV_CANFDSPI_ReceiveChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
			//debugPrintf("Recv message %d status %x op mode %d\n", whichBuffer, status, DRV_CANFDSPI_OperationModeGet(0));
			if (status & CAN_RX_FIFO_OVERFLOW)
			{
				debugPrintf("rx queue %d overflow status %x\n", whichBuffer, status);
				messagesLost++;
				DRV_CANFDSPI_ReceiveChannelEventOverflowClear(0, (CAN_FIFO_CHANNEL) whichBuffer);
			DRV_CANFDSPI_ReceiveChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
				debugPrintf("rx queue %d status2 %x\n", whichBuffer, status);
			}
			if (status & CAN_RX_FIFO_FULL) bufFullCnt[(int)whichBuffer]++;
			if (status & CAN_RX_FIFO_NOT_EMPTY)
			{
			//debugPrintf("Recv message %d status %x op mode %d\n", whichBuffer, status, DRV_CANFDSPI_OperationModeGet(0));
				CAN_RX_MSGOBJ rxObj;
				uint8_t status2;
				status2 = DRV_CANFDSPI_ReceiveMessageGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &rxObj, buffer->msg.raw, sizeof(buffer->msg.raw));
				CopyHeader(buffer, &rxObj);
			if (status & CAN_RX_FIFO_OVERFLOW)
			{
			DRV_CANFDSPI_ReceiveChannelStatusGet(0, (CAN_FIFO_CHANNEL) whichBuffer, &status);
				debugPrintf("rx queue %d status3 %x\n", whichBuffer, status);
			}
				//debugPrintf("src %d dst %d type %d len %d\n", buffer->id.Src(), buffer->id.Dst(), buffer->id.MsgType(), buffer->dataLength);
				bufReqCnt[(int)whichBuffer]++;
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
	debugPrintf("Set EFE index %d limit %d\n", index, config->numExtendedFilterElements);
	if (index < config->numExtendedFilterElements)
	{
		debugPrintf("Add filter index %d buffer %d id %x mask %x\n", index, whichBuffer, id, mask);
		MutexLocker lock(SPIMutex);
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
	{
		MutexLocker lock(SPIMutex);
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
	AtomicCriticalSectionLocker lock;

	rMessagesQueuedForSending = messagesQueuedForSending;
	rMessagesReceived = messagesReceived;
	rMessagesLost = messagesLost;
	rBusOffCount = busOffCount;
	messagesQueuedForSending = messagesReceived = messagesLost = busOffCount = 0;
}

#ifdef RTOS


#endif
#endif
#endif
