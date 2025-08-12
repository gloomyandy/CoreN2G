# This file is based on the Arduini rp2xxx build scripts, with
# modifications for RRF by Andy
cmake_minimum_required(VERSION 3.12)

set(cpu $ENV{CPU})
message("Building for CPU ${cpu}")
set(TINYUSB_OPT_OS OPT_OS_FREERTOS)
set(PICO_CYW43_SUPPORTED 0)

if (${cpu} MATCHES "rp2040")
	# Build fior original pico board without WiFi
	set(PICO_BOARD pico)
	set(PICO_PLATFORM rp2040)
elseif(${cpu} MATCHES "rp2350$")
	set(PICO_BOARD solderparty_rp2350_stamp_xl)  # Pico2 sets to RP2350A which disables all code for RP2350B
	set(PICO_PLATFORM rp2350)
elseif(${cpu} MATCHES "rp2350-riscv$")
	message(INFO "Building RISCV")
	set(PICO_BOARD solderparty_rp2350_stamp_xl)  # Pico2 sets to RP2350A which disables all code for RP2350B
	set(PICO_PLATFORM rp2350-riscv)
else()
	message(FATAL_ERROR "Unknown CPU, '${cpu}'")
endif()

include(pico_sdk_import.cmake)

project(pico_lib C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Initialize the SDK
pico_sdk_init()

add_library(common-${cpu} INTERFACE)

# Per CPU model custom flags
if (${cpu} MATCHES "rp2040$")
	set(xcd
		PICO_RP2040_B0_SUPPORTED=1
		PICO_RP2040_B1_SUPPORTED=1
		PICO_RP2040_B2_SUPPORTED=1
		PICO_FLOAT_SUPPORT_ROM_V1=1
		PICO_DOUBLE_SUPPORT_ROM_V1=1
		PICO_RP2040=1
		__RP2040__=1					# added
	)
elseif (${cpu} MATCHES "rp2350$")
	set(xcd
		PICO_BOARD=\"solderparty_rp2350_stamp_xl\"
		PICO_BUILD=1
		PICO_RP2350=1
		LIB_PICO_SHA256=1
		LIB_PICO_DOUBLE_PICO=1
		LIB_PICO_FLOAT_PICO=1
		LIB_PICO_FLOAT_PICO_VFP=1
		__RP2350__=1					# added
	)
elseif (${cpu} MATCHES "rp2350-riscv$")
	set(xcd
		PICO_BOARD=\"solderparty_rp2350_stamp_xl\"
		PICO_BUILD=1
		PICO_RP2350=1
		LIB_PICO_SHA256=1
		LIB_PICO_DOUBLE_COMPILER=1
		LIB_PICO_FLOAT_COMPILER=1
		PICO_RISCV=1
		__RP2350__=1					# added
	)
endif()

# defines which are automatically added by cmake when using target_link_libraries
# commented out by Andy
target_compile_definitions(common-${cpu} INTERFACE
	PICO_PLATFORM=${PICO_PLATFORM}

	PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
	PICO_PRINTF_ALWAYS_INCLUDED=0 # changed
	PICO_FLASH_SIZE_BYTES=16777216

	PICO_32BIT=1
	PICO_NO_BINARY_INFO=1
	PICO_NO_FLASH=0
	PICO_NO_HARDWARE=0
	PICO_ON_DEVICE=1

	PICO_COPY_TO_RAM=0
	PICO_CXX_ENABLE_EXCEPTIONS=0

	PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1
	PICO_RP2040_USB_DEVICE_UFRAME_FIX=1

	PICO_USE_BLOCKED_RAM=0

	LWIP_IPV4=0 #changed
	LWIP_UDP=0 #changed
	LWIP_IGMP=0 #changed
	LWIP_CHECKSUM_CTRL_PER_NETIF=0 #changed

	PICO_CYW43_SUPPORTED=0 #changed
	PICO_CYW43_ARCH_THREADSAFE_BACKGROUND=0 #changed
	CFG_TUSB_DEBUG=0
	CFG_TUSB_MCU=OPT_MCU_RP2040
	CFG_TUSB_OS=OPT_OS_FREERTOS #changed
	LIB_TINYUSB_BOARD=1
	LIB_TINYUSB_DEVICE=1

	CYW43_WARN=//
	CYW43_PIO_CLOCK_DIV_DYNAMIC=1
	CYW43_PIN_WL_DYNAMIC=1
	CYW43_DEFAULT_PIN_WL_REG_ON=23u
	CYW43_DEFAULT_PIN_WL_DATA_OUT=24u
	CYW43_DEFAULT_PIN_WL_DATA_IN=24u
	CYW43_DEFAULT_PIN_WL_HOST_WAKE=24u
	CYW43_DEFAULT_PIN_WL_CLOCK=29u
	CYW43_DEFAULT_PIN_WL_CS=25u
	#LIB_BOOT_STAGE2_HEADERS=1
	#LIB_PICO_ATOMIC=1
	#LIB_PICO_BIT_OPS=1
	#LIB_PICO_BIT_OPS_PICO=1
	#LIB_PICO_CLIB_INTERFACE=1
	#LIB_PICO_CRT0=1
	#LIB_PICO_CXX_OPTIONS=1
	#LIB_PICO_DIVIDER=1
	#LIB_PICO_DIVIDER_COMPILER=1

	LIB_PICO_DOUBLE=1
	#LIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1
	#LIB_PICO_FLOAT=1
	#LIB_PICO_INT64_OPS=1
	#LIB_PICO_INT64_OPS_COMPILER=1
	#LIB_PICO_MEM_OPS=1
	#LIB_PICO_MEM_OPS_COMPILER=1
	#LIB_PICO_MULTICORE=1
	#LIB_PICO_NEWLIB_INTERFACE=1
	#LIB_PICO_PLATFORM=1
	#LIB_PICO_PLATFORM_COMPILER=1
	#LIB_PICO_PLATFORM_PANIC=1
	#LIB_PICO_PLATFORM_SECTIONS=1
	#LIB_PICO_RUNTIME=1
	#LIB_PICO_RUNTIME_INIT=1
	#LIB_PICO_STANDARD_BINARY_INFO=1
	#LIB_PICO_STANDARD_LINK=1
	#LIB_PICO_SYNC=1
	#LIB_PICO_SYNC_CRITICAL_SECTION=1
	#LIB_PICO_SYNC_MUTEX=1
	#LIB_PICO_SYNC_SEM=1
	#LIB_PICO_TIME=1
	#LIB_PICO_TIME_ADAPTER=1
	#LIB_PICO_UNIQUE_ID=1
	#LIB_PICO_UTIL=1
	noexcept=	# added
	${xcd}
)

target_compile_options(common-${cpu} INTERFACE
	-fno-exceptions
	-Os
	$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)

# changed
include_directories(BEFORE ${PICO_SDK_PATH}/../tools/Build-pico-sdk)
include_directories(BEFORE ${PICO_SDK_PATH}/../FreeRTOSConfig)
include_directories(BEFORE ${PICO_SDK_PATH}/../../../../FreeRTOS/src/include)
if (${cpu} MATCHES "rp2040")
	include_directories(BEFORE ${PICO_SDK_PATH}/../../../../FreeRTOS/src/portable/GCC/ARM_CM0)
	elseif(${cpu} MATCHES "rp2350$")
	include_directories(BEFORE ${PICO_SDK_PATH}/../../../../FreeRTOS/src/portable/GCC/ARM_CM33/non_secure)
endif()

add_library(pico-${cpu} STATIC)
target_compile_definitions(pico-${cpu} PUBLIC
	LWIP_IPV6=0
)
if (${cpu} MATCHES "rp2040")
	set(xll1)
elseif(${cpu} MATCHES "rp2350")
	set(xll1
		pico_sha256)
endif()

target_link_libraries(pico-${cpu}
	common-${cpu}
	boot_stage2
	hardware_adc
	hardware_base
	hardware_claim
	hardware_clocks
	hardware_divider
	hardware_dma
	hardware_exception
	hardware_flash
	hardware_gpio
	hardware_i2c
	hardware_interp
	hardware_irq
	hardware_pio
	hardware_pll
	hardware_pwm
	hardware_resets
	hardware_rtc
	hardware_spi
	hardware_sync
	hardware_timer
	hardware_uart
	hardware_vreg
	hardware_watchdog
	hardware_xosc
	cmsis
	pico_aon_timer
	pico_bit_ops
	pico_bootrom
	pico_bootsel_via_double_reset
	pico_cxx_options
	pico_divider
	pico_double
	pico_fix
	pico_float
	pico_int64_ops
	pico_malloc
	pico_mem_ops
	pico_multicore
	pico_platform
	pico_rand
	pico_runtime
	pico_runtime_init
	pico_standard_link
	pico_stdlib
	pico_unique_id
	pico_util
	${xll1}
	tinyusb
	tinyusb_device_unmarked
)

foreach(tgt pico) # changed
	add_custom_command(TARGET ${tgt}-${cpu} POST_BUILD
		COMMAND ar d lib${tgt}-${cpu}.a stdio.c.obj stdio_uart.c.obj stdio_usb.c.obj stdio_usb_descriptors.c.obj pico_malloc.c.obj newlib_interface.c.obj
		COMMAND ar d lib${tgt}-${cpu}.a btstack_flash_bank.c.obj  # Need to override with our own implementation
		COMMAND cp lib${tgt}-${cpu}.a ../../../lib/${cpu}/lib${tgt}.a
	)
endforeach()
add_custom_command(TARGET pico-${cpu} POST_BUILD
	COMMAND mkdir -p ../../../include/${cpu}/pico_base/pico
	COMMAND cp ./generated/pico_base/pico/version.h ../../../include/${cpu}/pico_base/pico/.
	COMMAND sed 's/include.*pico-sdk/include \"..\\/..\\/..\\/..\\/pico-sdk/' ./generated/pico_base/pico/config_autogen.h > ../../../include/${cpu}/pico_base/pico/config_autogen.h
	COMMAND cp ../tusb_config.h ../../../include/${cpu}/.
	COMMAND cp ../btstack_config.h ../../../include/${cpu}/.
)
