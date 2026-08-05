#ifndef USB_INTERFACE_H
#define USB_INTERFACE_H
#ifdef __cplusplus
extern "C" {
#endif

void usb_init(void);
void usb_deinit(void);
uint32_t usb_write(const uint8_t *buf, uint32_t cnt);
uint32_t usb_available_for_write();
uint32_t usb_read(uint8_t *buf, uint32_t cnt);
uint32_t usb_available_for_read();
bool usb_is_connected();
bool usb_is_write_empty();
void usb_spin();
#ifdef __cplusplus
}
#endif

#endif
