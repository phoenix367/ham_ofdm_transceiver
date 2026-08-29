/* TinyUSB configuration for the OFDM modem.
 *
 * One vendor-class interface with two bulk endpoints. No CDC, no HID,
 * no MSC: the whole point of this device is to be identifiable, and
 * every extra interface is another thing for a host to bind a driver to.
 */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU            OPT_MCU_STM32H7
#define CFG_TUSB_OS             OPT_OS_NONE
/* Port 0 is USB2_OTG_FS (PA11/PA12) on an H743; port 1 is USB1_OTG_HS.
 * Overridable so the same source builds for a board wired the other
 * way -- the only differences are this, the pin AF and the RCC bit. */
#ifndef OFDM_USB_RHPORT
#define OFDM_USB_RHPORT 0
#endif
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_RHPORT1_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

/* No special section. TinyUSB's STM32 dwc2 port runs the core in
 * slave/FIFO mode, not DMA: the CPU copies packets into and out of the
 * peripheral's own FIFOs, so these buffers are ordinary memory and can
 * live in .bss like anything else. Pinning them to a particular SRAM
 * would add a clock-gating and cache-coherency problem to solve for no
 * benefit. If DMA mode is ever enabled, that changes -- the buffers
 * then need a DMA-reachable, non-cached region, and this is where to
 * say so. */
#define CFG_TUSB_MEM_ALIGN      __attribute__((aligned(4)))

#define CFG_TUD_ENABLED         1
#define CFG_TUD_ENDPOINT0_SIZE  64

#define CFG_TUD_CDC             0
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          1

/* Deep enough that a host polling at 1 ms never starves the modem, and
 * shallow enough to stay irrelevant next to the receiver's buffers. */
#define CFG_TUD_VENDOR_RX_BUFSIZE 512
#define CFG_TUD_VENDOR_TX_BUFSIZE 512

#endif /* TUSB_CONFIG_H */
