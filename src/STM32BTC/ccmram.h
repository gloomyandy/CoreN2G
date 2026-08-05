#ifndef CCMRAM_H
#define CCMRAM_H
#ifdef __cplusplus
extern const char *ccmHeapLimit;
extern char *ccmHeapTop;
extern char _eccmram;

void *CoreAllocCCMRAMPermanent(size_t sz, std::align_val_t align) noexcept;
void CoreCCMRAMUsage(size_t& ccmStatic, size_t& ccmUsed, size_t& ccmFree) noexcept;
#endif
#endif
