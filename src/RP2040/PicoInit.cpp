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
    SystemCoreClock = clock_get_hz(clk_sys);
}