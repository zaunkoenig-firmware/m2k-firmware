#include <assert.h>
#include <stdint.h>
#include "stm32f7xx.h"
#include "usbd_hid.h"
#include "usb.h"
#include "anim.h"
#include "btn_whl.h"
#include "clock.h"
#include "config.h"
#include "delay.h"
#include "m2k_resource.h"
#include "pmw3360.h"

#define TIMEOUT_SECS 5 // seconds of holding buttons for programming mode

typedef union {
	struct __PACKED { // use the order in the report descriptor
		uint8_t btn;
		int8_t whl;
		int16_t x, y;
		uint16_t _pad; // zero pad to 8 bytes total
	};
	uint8_t u8[8]; // btn, wheel, xlo, xhi, ylo, yhi, 0, 0
	uint32_t u32[2];
} Usb_packet;
static_assert(sizeof(Usb_packet) == 2*sizeof(uint32_t));

static Config config_boot(void)
{
	// read button state on boot
	uint8_t btn_boot = 0;
	btn_boot |= (!(LMB_NO_PORT->IDR & LMB_NO_PIN)) << 0;
	btn_boot |= (!(RMB_NO_PORT->IDR & RMB_NO_PIN)) << 1;
	// if both L+R are pressed, bootloader waits until one of them is released.
	// so by the time we read IDR above, at least one button will be released.
	// solution: check SysTick_CTRL ENABLE flag, which would be set in that case.
	if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0) {
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
		btn_boot |= 0b11;
	}

	// update config depending on initial buttons
	delay_ms(25); // delay in case power bounces on boot
	Config cfg = config_read();
	switch (btn_boot) {
	case 0b01: // LMB pressed
		cfg.flags ^= CONFIG_FLAGS_ANGLE_SNAP_ON;
		config_write(cfg);
		if (cfg.flags & CONFIG_FLAGS_ANGLE_SNAP_ON)
			anim_cw(1);
		else
			anim_ccw(1);
		break;
	case 0b10: // RMB pressed
		cfg.flags ^= CONFIG_FLAGS_3MM_LOD;
		config_write(cfg);
		if (cfg.flags & CONFIG_FLAGS_3MM_LOD)
			anim_cw(3);
		else
			anim_ccw(2);
		break;
	case 0b11: // LMB and RMB pressed
		cfg.flags ^= CONFIG_FLAGS_HS_USB;
		config_write(cfg);
		if (cfg.flags & CONFIG_FLAGS_HS_USB)
			anim_eight(1);
		else
			anim_updown(1);
		break;
	}
	return cfg;
}

static inline uint32_t mode_process(
		Config *cfg,
		const Usb_packet *usb_pkt,
		const Usb_packet *usb_pkt_prev,
		uint8_t squal)
{
	// mode 0: normal
	// mode 1: cpi programming
	// mode 2: Hz programming
	// mode 4: normal after releasing L+R (prev: cpi programming)
	// mode 5: cpi programming after releasing L+R
	// mode 6: Hz programming after releasing MMB
	// mode 8: normal after releasing L+R (prev: Hz programming)
	static uint32_t mode = 0;
	static uint32_t ticks = 0; // counter for programming mode timeout

	if (mode == 1) { // handle cpi mode
		const uint8_t released = (~usb_pkt->btn) & usb_pkt_prev->btn;
		if ((released & 0b01) != 0 && cfg->dpi > 0x00) { // LMB released
			cfg->dpi--;
			anim_left(1);
			pmw3360_set_dpi(cfg->dpi);
		}
		if ((released & 0b10) != 0 && cfg->dpi < 0x77) { // RMB released
			cfg->dpi++;
			anim_right(1);
			pmw3360_set_dpi(cfg->dpi);
		}
	} else if (mode == 2) { // handle Hz mode
		if (usb_pkt->whl > 0 &&
				(cfg->flags & CONFIG_FLAGS_INTERVAL) > 0) {
			cfg->flags -= 1 << CONFIG_FLAGS_INTERVAL_Pos;
			anim_up(1);
		} else if (usb_pkt->whl < 0 &&
				(cfg->flags & CONFIG_FLAGS_INTERVAL) < CONFIG_FLAGS_INTERVAL) {
			cfg->flags += 1 << CONFIG_FLAGS_INTERVAL_Pos;
			anim_down(1);
		}
	}
	if (squal < 16) { // not tracking, check if right buttons are held
		const int timeout_ticks = TIMEOUT_SECS * \
				((cfg->flags & CONFIG_FLAGS_HS_USB) != 0 ? 8000 : 1000);
		if (usb_pkt->btn == 0b011) { // only L+R held
			if (mode == 0 || mode == 1) {
				ticks++;
				if (ticks >= timeout_ticks)
					mode = (mode == 0) ? 5 : 4;
			}
		} else if (usb_pkt->btn == 0b100) { // only M held
			if (mode == 0 || mode == 2) {
				ticks++;
				if (ticks >= timeout_ticks)
					mode = (mode == 0) ? 6 : 8;
			}
		} else {
			ticks = 0;
		}
	} else {
		ticks = 0;
	}
	// enter programming mode after releasing buttons in waiting mode
	if (mode >= 4 && usb_pkt->btn == 0b000) {
		if (mode == 4 || mode == 5) { // released L+R
			anim_rightleft((cfg->dpi + 1)/10); // show dpi
			anim_updown((cfg->dpi + 1)%10);
		} else if (mode == 6 || mode == 8) { // released M
			anim_updown(8 >> _FLD2VAL(CONFIG_FLAGS_INTERVAL, cfg->flags)); // show Hz
		}
		mode &= 0b11;
		if (mode == 0) // save config when returning to normal mode
			config_write(*cfg);
	}
	return mode;
}

int main(void)
{
//	extern uint32_t _sitcm;
//	SCB->VTOR = (uint32_t)(&_sitcm);
	extern uint32_t _sflash;
	SCB->VTOR = (uint32_t)(&_sflash);
	SCB_EnableICache();
	SCB_EnableDCache();

	clk_init();
	delay_init();
	btn_init();
	whl_init();
	int whl_prev_same = 0, whl_prev_diff = 0;

	Config cfg = config_boot();

	const int hs_usb = ((cfg.flags & CONFIG_FLAGS_HS_USB) != 0);
	usb_init(hs_usb);
	usb_wait_configured();

	spi_init();
	pmw3360_init(cfg);

	const uint32_t USBx_BASE = (uint32_t)USB_OTG_HS; // used in macros USBx_*
	// fifo space when empty, should equal 0x174, from init_usb
	const uint32_t fifo_space = (USBx_INEP(1)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV);

	Usb_packet usb_pkt = {0}, usb_pkt_prev = {0};

	USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_SOFM; // enable SOF interrupt
	while (1) {
		// always check that usb is configured
		usb_wait_configured();

		// wait for SOF to sync to usb frames
		USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_SOF;
		__WFI();

		// if full speed usb, delay here to minimize input lag
		if (!hs_usb) delay_us(875);

		// read sensor, wheel, buttons
		ss_low();
		spi_send(0x50);
		delay_us(35);
		(void)spi_recv(); // motion, not used
		(void)spi_recv(); // observation, not used
		usb_pkt.u8[2] = spi_recv(); // x lower 8 bits
		usb_pkt.u8[3] = spi_recv(); // x upper 8 bits
		usb_pkt.u8[4] = spi_recv(); // y lower 8 bits
		usb_pkt.u8[5] = spi_recv(); // y upper 8 bits
		const uint8_t squal = spi_recv(); // SQUAL
		ss_high();

		usb_pkt.whl = 0;
		const int whl_p = whl_read_p();
		const int whl_n = whl_read_n();
		if (whl_p != whl_n)
			whl_prev_diff = whl_p;
		else if (whl_p != whl_prev_same) {
			whl_prev_same = whl_p;
			usb_pkt.whl = 2 * (whl_p ^ whl_prev_diff) - 1;
		}

		const uint16_t btn_raw = btn_read();
		const uint8_t btn_NO = (btn_raw & 0xFF);
		const uint8_t btn_NC = (btn_raw >> 8);
		usb_pkt.btn = (~btn_NO & 0b111) | (btn_NC & usb_pkt_prev.btn);

		// mode processing
		const uint32_t mode = mode_process(&cfg, &usb_pkt, &usb_pkt_prev, squal);
		// mask to block inputs in programming modes
		const uint32_t mode_mask[3] = {0xffffffff, 0xffffff00, 0xffff00ff};

		// animation stuff
		struct Xy a = anim_read(); // returns 0 if no animation left
		usb_pkt.x += a.x;
		usb_pkt.y += a.y;

		// if last packet not transmitted yet
		if ((USBx_INEP(1)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) < fifo_space) {
			// flush fifo
			USB_OTG_HS->GRSTCTL = _VAL2FLD(USB_OTG_GRSTCTL_TXFNUM, 1) | USB_OTG_GRSTCTL_TXFFLSH;
			while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) != 0);
			// combine flushed data with current data
			usb_pkt.btn |= usb_pkt_prev.btn; // a pulse still counts as a click
			usb_pkt.whl += usb_pkt_prev.whl;
			usb_pkt.x   += usb_pkt_prev.x;
			usb_pkt.y   += usb_pkt_prev.y;
		}

		// disable transfer complete interrupts, in case enabled by OTG_HS_IRQHandler
		USBx_DEVICE->DIEPMSK &= ~USB_OTG_DIEPMSK_XFRCM;

		// if there is data to transmitted
		if (usb_pkt.btn != usb_pkt_prev.btn || usb_pkt.whl || usb_pkt.x || usb_pkt.y) {
			// set up transfer size
			MODIFY_REG(USBx_INEP(1)->DIEPTSIZ,
					USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
					_VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, HID_EPIN_SIZE)
			);
			// enable endpoint
			USBx_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
			// write to fifo
			USBx_DFIFO(1) = usb_pkt.u32[0] & mode_mask[mode & 0b11];
			USBx_DFIFO(1) = usb_pkt.u32[1];
			usb_pkt_prev = usb_pkt;
		}
	}
	return 0;
}
