/*
 * Flash.h
 *
 *  Created on: 8 Aug 2019
 *      Author: David
 */

#ifndef SRC_FLASH_H_
#define SRC_FLASH_H_

#include <CoreIO.h>

namespace Flash
{
	bool Init() noexcept;
	bool Unlock(uint32_t start, uint32_t length) noexcept;
	bool Erase(uint32_t start, uint32_t length) noexcept;
	bool Lock(uint32_t start, uint32_t length) noexcept;
	bool Write(uint32_t start, uint32_t length, uint8_t *data) noexcept;

#if SAMC21
	bool RwwErase(uint32_t start, uint32_t length) noexcept;
	bool RwwWrite(uint32_t start, uint32_t length, const uint8_t *data) noexcept;
#endif

	uint32_t GetPageSize() noexcept;
	uint32_t GetLockRegionSize() noexcept;
	uint32_t GetEraseRegionSize() noexcept;
	uint32_t GetFlashSize() noexcept;
}

extern "C" uint32_t GetFlashSize_C() noexcept;

#endif /* SRC_FLASH_H_ */
