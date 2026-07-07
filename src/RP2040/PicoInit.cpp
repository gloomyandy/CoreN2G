#include "pico/runtime_init.h"
#include "hardware/clocks.h"

uint32_t SystemCoreClock = SYS_CLK_HZ;

void runtime_init(void) {
#ifndef NDEBUG
    if (__get_current_exception()) {
        // crap; started in exception handler
        __breakpoint();
    }
#endif

#if !PICO_RUNTIME_SKIP_INIT_PER_CORE_INSTALL_STACK_GUARD
    // install core0 stack guard
    extern char __StackBottom;
    runtime_init_per_core_install_stack_guard(&__StackBottom);
#endif

    // todo maybe we want to do this in the future, but it does stuff like register_tm_clones
    //      which we didn't do in previous SDKs
    //extern void __libc_init_array(void);
    //__libc_init_array();

    // ... so instead just do the __preinit_array
    runtime_run_initializers();
    // ... and the __init_array
    extern void (*__init_array_start)(void);
    extern void (*__init_array_end)(void);
    for (void (**p)(void) = &__init_array_start; p < &__init_array_end; ++p) {
        (*p)();
    }
#if defined(__RP2350__) && defined(BOARD_OVERCLOCK_SYS_KHZ)
    // Raise the system clock above the stock 150MHz. The SDK runtime library is prebuilt with the stock
    // clock configuration, so the PLL must be reprogrammed at runtime after the standard initialisers
    // have run. clk_peri follows clk_sys so that peripheral (SPI/UART) baud dividers stay correct.
    set_sys_clock_khz(BOARD_OVERCLOCK_SYS_KHZ, true);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    BOARD_OVERCLOCK_SYS_KHZ * 1000u, BOARD_OVERCLOCK_SYS_KHZ * 1000u);
#endif
    SystemCoreClock = clock_get_hz(clk_sys);
}