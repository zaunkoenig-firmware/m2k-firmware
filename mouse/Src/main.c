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
static_assert(sizeof(Usb_packet) == 2*sizeof(uint32_t), "Usb_packet wrong size");

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
			anim_one(1);
		break;
	}
	return cfg;
}
/*
 static Config config_boot(void)
{
	// read button state on boot
	uint8_t btn_boot = 0;
	btn_boot |= (!(LMB_NO_PORT->IDR & LMB_NO_PIN)) << 0;
	btn_boot |= (!(RMB_NO_PORT->IDR & RMB_NO_PIN)) << 1;
	btn_boot |= (!(MMB_NO_PORT->IDR & MMB_NO_PIN)) << 2;
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
	case 0b100: // MMB pressed
		cfg.flags ^= CONFIG_FLAGS_ANGLE_SNAP_ON;
		config_write(cfg);
		if (cfg.flags & CONFIG_FLAGS_ANGLE_SNAP_ON)
			anim_cw(1);
		else
			anim_ccw(1);
		break;
	case 0b001: // LMB pressed
		cfg.flags ^= CONFIG_FLAGS_3MM_LOD;
		config_write(cfg);
		if (cfg.flags & CONFIG_FLAGS_3MM_LOD)
			anim_cw(3);
		else
			anim_ccw(2);
		break;
	case 0b011: // LMB and RMB pressed
		cfg.flags ^= CONFIG_FLAGS_HS_USB;
		config_write(cfg);
		if (cfg.flags & CONFIG_FLAGS_HS_USB)
			anim_eight(1);
		else
			anim_one(1);
		break;
	}
	return cfg;
}
 */

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))


static inline uint32_t mode_process(Config *cfg, int *skip,
		uint8_t *holding_transitioned,
		uint8_t *large_step,
		const uint8_t btn, const uint8_t btn_prev,
		const int8_t whl, const uint8_t squal)
{
	// mode 0: normal
	// mode 1: cpi programming
	// mode 2: Hz programming (only available in HS USB mode)
	// mode 4: normal after releasing L+R (prev: cpi programming)
	// mode 5: cpi programming after releasing L+R
	// mode 6: Hz programming after releasing MMB
	// mode 8: normal after releasing L+R (prev: Hz programming)
	static uint32_t mode = 0;
	static uint32_t ticks = 0; // counter for programming mode timeout

	// if the user's continued to hold buttons after a state transition has happened, don't act on those clicks as if they were new
	if(!*holding_transitioned) {
		if (mode == 1) { // handle cpi mode
			const uint8_t released = (~btn) & btn_prev;
			if ((released & 0b01) != 0 && cfg->dpi > 0x00) { // LMB released
				if (btn & 0b10) {
					cfg->dpi = MAX(cfg->dpi - 10, 0);
					anim_lg_downup(1);
					*large_step = 1;
				} else {
					if(!*large_step){
						cfg->dpi-=1;
						anim_downup(1);
					} else {
						*large_step = 0;
					}
				}
				pmw3360_set_dpi(cfg->dpi);
			}
			if ((released & 0b10) != 0 && cfg->dpi < 0x77) { // RMB released
				if (btn & 0b01) {
					cfg->dpi+=10;
					anim_lg_updown(1);
					*large_step = 1;
				} else {
					if (!*large_step){
						cfg->dpi++;
						anim_updown(1);
					} else {
						*large_step = 0;
					}
				}


				pmw3360_set_dpi(cfg->dpi);
			}
		} else if (mode == 2) { // handle Hz mode
			if (whl > 0 &&
					(cfg->flags & CONFIG_FLAGS_INTERVAL) > 0) {
				cfg->flags -= 1 << CONFIG_FLAGS_INTERVAL_Pos;
				int itv = _FLD2VAL(CONFIG_FLAGS_INTERVAL, cfg->flags);
				*skip = (1 << itv) - 1;

				int prev_hz = 8 >> (itv+1);
				anim_updown((8 >> itv) - prev_hz);
			} else if (whl < 0 &&
					(cfg->flags & CONFIG_FLAGS_INTERVAL) < CONFIG_FLAGS_INTERVAL) {
				cfg->flags += 1 << CONFIG_FLAGS_INTERVAL_Pos;
				int itv = _FLD2VAL(CONFIG_FLAGS_INTERVAL, cfg->flags);
				*skip = (1 << itv) - 1;

				int prev_hz = 8 >> (itv-1);
				anim_downup(prev_hz - (8 >> itv));
			}
		}

		if (squal < 16) { // not tracking, check if right buttons are held
			const int hs = ((cfg->flags & CONFIG_FLAGS_HS_USB) != 0);
			const int timeout_ticks = TIMEOUT_SECS * (hs ? 8000 : 1000);
			if (btn == 0b011) { // only L+R held
				if (mode == 0 || mode == 1) {
					ticks++;
					if (ticks >= timeout_ticks)
						mode = (mode == 0) ? 5 : 4;
				}
			} else if (hs && btn == 0b100) { // HS mode and only M held
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
	} else if (btn == 0) {
		// when the user lets go of their buttons, then the next cycle can process the state logic
		*holding_transitioned = 0;
	}

	// enter/exit programming mode
	if (mode >= 4) {
		if (mode == 4 || mode == 5) { // released L+R
			anim_rightleft((cfg->dpi + 1)/10); // show dpi
			anim_updown((cfg->dpi + 1)%10);
		} else if (mode == 6 || mode == 8) { // released M
			int itv = _FLD2VAL(CONFIG_FLAGS_INTERVAL, cfg->flags);
			anim_updown(8 >> itv); // show Hz
		}

		mode &= 0b11;

		// track whether the user's continued holding the buttons even after a state transition
		*holding_transitioned = btn;

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
	uint8_t btn_prev = 0;
	whl_init();
	int whl_prev_same = whl_read_p(), whl_prev_diff = whl_read_p();

	Config cfg = config_boot();

	const int hs_usb = ((cfg.flags & CONFIG_FLAGS_HS_USB) != 0);
	anim_set_scale(hs_usb ? 8 : 1);
	usb_init(hs_usb);
	usb_wait_configured();

	spi_init();
	pmw3360_init(cfg);

	const uint32_t USBx_BASE = (uint32_t)USB_OTG_HS; // used in macros USBx_*
	// fifo space when empty, should equal 0x174, from init_usb
	const uint32_t fifo_space = (USBx_INEP(1)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV);

	Usb_packet new = {0}; // what's new this loop
	Usb_packet send = {0}; // what's transmitted

	int skip = hs_usb ? (1 << _FLD2VAL(CONFIG_FLAGS_INTERVAL, cfg.flags)) - 1 : 0;
	int count = 0; // counter to skip reports
	uint8_t holding_transitioned = 0; // mbutton hold state for config
	uint8_t large_step = 0;

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
		new.u8[2] = spi_recv(); // x lower 8 bits
		new.u8[3] = spi_recv(); // x upper 8 bits
		new.u8[4] = spi_recv(); // y lower 8 bits
		new.u8[5] = spi_recv(); // y upper 8 bits
		const uint8_t squal = spi_recv(); // SQUAL
		ss_high();

		new.whl = 0;
		const int whl_p = whl_read_p();
		const int whl_n = whl_read_n();
		if (whl_p != whl_n)
			whl_prev_diff = whl_p;
		else if (whl_p != whl_prev_same) {
			whl_prev_same = whl_p;
			new.whl = 2 * (whl_p ^ whl_prev_diff) - 1;
		}

		const uint16_t btn_raw = btn_read();
		const uint8_t btn_NO = (btn_raw & 0xFF);
		const uint8_t btn_NC = (btn_raw >> 8);
		btn_prev = new.btn;
		new.btn = (~btn_NO & 0b111) | (btn_NC & btn_prev);

		// mode processing
		const uint32_t mode = mode_process(&cfg, &skip, &holding_transitioned, &large_step, new.btn, btn_prev, new.whl, squal);
		// mask to block inputs in programming modes
		const uint32_t mode_mask[3] = {0xffffffff, 0xffffff00, 0xffff00ff};

		// animation stuff
		const struct Xy a = anim_read(); // returns 0 if no animation left
		new.x += a.x;
		new.y += a.y;

		// if last packet still sitting in fifo
		if ((USBx_INEP(1)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) < fifo_space) {
			// flush fifo
			USB_OTG_HS->GRSTCTL = _VAL2FLD(USB_OTG_GRSTCTL_TXFNUM, 1) | USB_OTG_GRSTCTL_TXFFLSH;
			while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) != 0);
			count = 0; // reset counter, try to transmit again
		} else if (count == skip) { // last loop transmitted successfully
			send.whl = 0;
			send.x = 0;
			send.y = 0;
		}
		send.whl += new.whl;
		send.x += new.x;
		send.y += new.y;

		// skip transmission for "skip" loops after a successful transmission
		if (count > 0) {
			count--;
			continue;
		}

		// if there is data to transmitted
		if (new.btn != send.btn || send.whl || send.x || send.y) {
			send.btn = new.btn;
			// disable transfer complete interrupts, in case enabled by OTG_HS_IRQHandler
			USBx_DEVICE->DIEPMSK &= ~USB_OTG_DIEPMSK_XFRCM;
			// set up transfer size
			MODIFY_REG(USBx_INEP(1)->DIEPTSIZ,
					USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
					_VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, HID_EPIN_SIZE)
			);
			// enable endpoint
			USBx_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
			// write to fifo
			USBx_DFIFO(1) = send.u32[0] & mode_mask[mode & 0b11];
			USBx_DFIFO(1) = send.u32[1];
			count = skip;
		}
	}
	return 0;
}
