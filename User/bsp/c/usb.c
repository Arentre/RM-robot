/* Includes ----------------------------------------------------------------- */
#include "bsp\usb.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bsp\delay.h"
#include "usbd_cdc_if.h"

/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private typedef ---------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
/* Must set to NULL explicitly. */
osThreadId_t gbsp_usb_alert = NULL;
uint8_t usb_rx_buf[BSP_USB_MAX_RX_LEN];
uint8_t usb_tx_buf[BSP_USB_MAX_TX_LEN];

/* Private function  -------------------------------------------------------- */
static int8_t BSP_USB_Transmit(uint8_t *buffer, uint16_t len) {
  uint8_t retry = 0;
  do {
    uint8_t result = CDC_Transmit_FS(buffer, len);
    if (result == USBD_OK) return BSP_OK;
    if (result == USBD_BUSY) {
      retry++;
      BSP_Delay(10);
    } else {
      return BSP_ERR_NO_DEV;
    }
  } while (retry < 3);
  return BSP_ERR;
}

/* Exported functions ------------------------------------------------------- */
int8_t BSP_USB_ReadyReceive(osThreadId_t alert) {
  if (alert == NULL) return BSP_ERR_NULL;

  gbsp_usb_alert = alert;
  return CDC_ReadyReceive() == USBD_OK ? BSP_OK : BSP_ERR_NO_DEV;
}

char BSP_USB_ReadChar(void) { return usb_rx_buf[0]; }

int8_t BSP_USB_Printf(const char *fmt, ...) {
  va_list ap;
  int len = 0;

  va_start(ap, fmt);
  len =
      vsnprintf((char *)usb_tx_buf, BSP_USB_MAX_TX_LEN, fmt, ap);
  va_end(ap);

  if (len > 0) {
    /* vsnprintf returns the required size, which may exceed the buffer. */
    if (len >= BSP_USB_MAX_TX_LEN) len = BSP_USB_MAX_TX_LEN - 1;
    return BSP_USB_Transmit(usb_tx_buf, (uint16_t)len);
  } else {
    return BSP_ERR_NULL;
  }
}
