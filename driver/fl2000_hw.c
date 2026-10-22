// SPDX-License-Identifier: GPL-2.0
/*
 * FL2000DX hardware access: EP0 register protocol, I2C master, mode setting.
 *
 * All sequences and table values come from Fresco Logic's GPL reference
 * driver (fl2000_monitor.c / fl2000_dongle.c / fl2000_big_table.c) and were
 * validated against real hardware from userspace first.
 */
#include <linux/delay.h>
#include <linux/usb.h>

#include "fl2000_drm.h"

#define FL2000_REQ_REG_READ	64
#define FL2000_REQ_REG_WRITE	65
#define FL2000_CTRL_TIMEOUT_MS	2000

#define FL2000_REG_I2C_CTRL	0x8020
#define FL2000_REG_I2C_RDATA	0x8024
#define FL2000_REG_I2C_WDATA	0x8028

static int fl2000_reg_read_locked(struct fl2000 *fl, u16 offset, u32 *val)
{
	__le32 v = 0;
	int ret;

	ret = usb_control_msg_recv(fl->udev, 0, FL2000_REQ_REG_READ,
				   USB_DIR_IN | USB_TYPE_VENDOR |
				   USB_RECIP_DEVICE,
				   0, offset, &v, sizeof(v),
				   FL2000_CTRL_TIMEOUT_MS, GFP_KERNEL);
	*val = le32_to_cpu(v);
	return ret;
}

static int fl2000_reg_write_locked(struct fl2000 *fl, u16 offset, u32 val)
{
	__le32 v = cpu_to_le32(val);

	return usb_control_msg_send(fl->udev, 0, FL2000_REQ_REG_WRITE,
				    USB_DIR_OUT | USB_TYPE_VENDOR |
				    USB_RECIP_DEVICE,
				    0, offset, &v, sizeof(v),
				    FL2000_CTRL_TIMEOUT_MS, GFP_KERNEL);
}

int fl2000_reg_read(struct fl2000 *fl, u16 offset, u32 *val)
{
	int ret;

	mutex_lock(&fl->hw_lock);
	ret = fl2000_reg_read_locked(fl, offset, val);
	mutex_unlock(&fl->hw_lock);
	return ret;
}

/*
 * Non-blocking variant for the streaming worker: hw_lock is also held
 * across slow DDC/EDID sequences (bus-hang aborts can take seconds), and
 * the stream must never wait on those or it starves the pixel pipe.
 * Returns -EBUSY when the lock is busy so the caller can skip the sample.
 */
int fl2000_reg_read_try(struct fl2000 *fl, u16 offset, u32 *val)
{
	int ret;

	if (!mutex_trylock(&fl->hw_lock))
		return -EBUSY;
	ret = fl2000_reg_read_locked(fl, offset, val);
	mutex_unlock(&fl->hw_lock);
	return ret;
}

int fl2000_reg_write(struct fl2000 *fl, u16 offset, u32 val)
{
	int ret;

	mutex_lock(&fl->hw_lock);
	ret = fl2000_reg_write_locked(fl, offset, val);
	mutex_unlock(&fl->hw_lock);
	return ret;
}

static int fl2000_reg_update_locked(struct fl2000 *fl, u16 offset, u32 clear,
				    u32 set)
{
	u32 val;
	int ret;

	ret = fl2000_reg_read_locked(fl, offset, &val);
	if (ret)
		return ret;
	val &= ~clear;
	val |= set;
	return fl2000_reg_write_locked(fl, offset, val);
}

/*
 * One 4-byte transaction on the FL2000 I2C master (regs 0x8020/24/28).
 * Control layout: [6:0] addr, [7] rw, [15:8] target offset, [16] spi op,
 * [17] spi erase, [27:24] per-byte status, [31] done. Bit 28 reads back 0
 * but must be written as 1 (hardware quirk noted by the reference driver).
 */
static int fl2000_i2c_xfer_locked(struct fl2000 *fl, u8 addr, u8 offset,
				  bool read, u32 *val)
{
	u32 ctrl;
	int ret, retry;

	if (!read) {
		ret = fl2000_reg_write_locked(fl, FL2000_REG_I2C_WDATA, *val);
		if (ret)
			return ret;
	}

	ret = fl2000_reg_read_locked(fl, FL2000_REG_I2C_CTRL, &ctrl);
	if (ret)
		return ret;

	ctrl |= BIT(28);
	ctrl &= ~(BIT(31) | BIT(17) | BIT(16) | 0xFFFF);
	ctrl |= addr & 0x7F;
	ctrl |= (read ? 1 : 0) << 7;
	ctrl |= offset << 8;

	ret = fl2000_reg_write_locked(fl, FL2000_REG_I2C_CTRL, ctrl);
	if (ret)
		return ret;

	/* ~0.9 ms on the wire at 100 kHz for addr + offset + 4 bytes */
	usleep_range(500, 800);

	for (retry = 0; retry < 100; retry++) {
		ret = fl2000_reg_read_locked(fl, FL2000_REG_I2C_CTRL, &ctrl);
		if (ret)
			return ret;
		if (ctrl & BIT(31)) {
			if ((ctrl >> 24) & 0xF)
				return -EREMOTEIO; /* NAK on some byte */
			if (read)
				return fl2000_reg_read_locked(fl,
						FL2000_REG_I2C_RDATA, val);
			return 0;
		}
		usleep_range(200, 400);
	}
	return -ETIMEDOUT;
}

int fl2000_i2c_read32(struct fl2000 *fl, u8 addr, u8 offset, u32 *val)
{
	int ret;

	mutex_lock(&fl->hw_lock);
	ret = fl2000_i2c_xfer_locked(fl, addr, offset, true, val);
	mutex_unlock(&fl->hw_lock);
	return ret;
}

int fl2000_i2c_write32(struct fl2000 *fl, u8 addr, u8 offset, u32 val)
{
	int ret;

	mutex_lock(&fl->hw_lock);
	ret = fl2000_i2c_xfer_locked(fl, addr, offset, false, &val);
	mutex_unlock(&fl->hw_lock);
	return ret;
}

/* Soft reset ("app reset", self clearing) */
int fl2000_hw_reset(struct fl2000 *fl)
{
	int ret;

	mutex_lock(&fl->hw_lock);
	ret = fl2000_reg_update_locked(fl, 0x8048, 0, BIT(15));
	mutex_unlock(&fl->hw_lock);
	return ret;
}

/*
 * Post-reset dongle init from the reference driver's card initialize:
 * enable the monitor/EDID detect machinery in 0x8020 (bits 28 and 30;
 * bit 28 always reads back 0, see the I2C quirk above). Without these,
 * VGA-only dongles (no IT66121) never drive their internal DAC and the
 * monitor sees no sync — found and validated on hardware by @ftoledo
 * (issue #1). Only for VGA dongles: on an IT66121 dongle these bits
 * corrupted the 0x8004 format latch (phantom readback bits) and the
 * bulk pipe stalled NAK-forever at 1920x1080 (-ETIMEDOUT).
 */
int fl2000_hw_dongle_init(struct fl2000 *fl)
{
	int ret;

	mutex_lock(&fl->hw_lock);
	ret = fl2000_reg_update_locked(fl, 0x8020, 0, BIT(30) | BIT(28));
	mutex_unlock(&fl->hw_lock);
	return ret;
}

/*
 * Timing/PLL table, bulk transfer path, from big_table_*_r0. Timings and
 * PLL are shared between the 16/24-bit variants of each mode.
 */
static const struct fl2000_mode fl2000_modes[] = {
	{  640,  480, 60, 0x2800320, 0x0600089, 0x1E0020D, 0x1C2001C, 0x003F6119,  1, 0x2, 0x19 },
	{  800,  600, 60, 0x3200420, 0x08000D9, 0x2580274, 0x1C4001C, 0x00080102,  0, 0x4, 0x17 },
	{ 1024,  768, 60, 0x4000540, 0x0880129, 0x3000326, 0x2460024, 0x000D2102,  0, 0x6, 0x1D },
	{ 1280,  720, 60, 0x5000672, 0x0280105, 0x2D002EE, 0x1F5001F, 0x00346107,  4, 0x5, 0x19 },
	{ 1366,  768, 60, 0x5560700, 0x08F0165, 0x300031E, 0x1C3001C, 0x00676206,  0, 0x3, 0x18 },
	/*
	 * 1360-wide variant of the entry above (same totals/PLL, 6 fewer
	 * active pixels): VGA-panel EDIDs advertise 1360x768 and their
	 * ADCs only sample cleanly against that preset.
	 */
	{ 1360,  768, 60, 0x5500700, 0x08F0165, 0x300031E, 0x1C3001C, 0x00676206,  0, 0x3, 0x18 },
	{ 1440,  900, 60, 0x5A00770, 0x0980181, 0x38403A6, 0x2060020, 0x00406106,  0, 0x6, 0x19 },
	{ 1280, 1024, 60, 0x5000698, 0x0700169, 0x400042A, 0x2A3002A, 0x00616109,  0, 0x3, 0x26 },
	{ 1680, 1050, 60, 0x69008C0, 0x0B001C9, 0x41A0441, 0x2560025, 0x00756204,  0, 0x6, 0x1E },
	{ 1920, 1080, 60, 0x7800898, 0x02C00C1, 0x4380465, 0x2A5002A, 0x00596106, 16, 0x5, 0x24 },
	{ 1600, 1200, 60, 0x6400870, 0x0C001F1, 0x4B004E2, 0x3230032, 0x00616106,  0, 0x3, 0x2E },
	/*
	 * 2560x1080@60, synthesized from an LG ULTRAWIDE's native DTD
	 * (185.58 MHz, 2784x1111 totals) — not from the reference big
	 * table. PLL = 10 MHz x 93 / 5 = 186.0 MHz (+0.23%, 60.13 Hz).
	 * Register encodings validated by re-encoding the CEA 1080p60
	 * row bit-exactly; PLL model from the reference (VCO 62.5M-1G,
	 * prescaler/multiplier/divisor, function = VCO band). The chip
	 * scans faster clocks in the official table (1920x1440@60 =
	 * 234 MHz), and at RGB565 this line width needs fewer bytes per
	 * line than the official 1920@24bpp modes.
	 */
	{ 2560, 1080, 60, 0xA000AE0, 0x04000A1, 0x4380457, 0x1DA001D, 0x005D6105,  0, 0xA, 0x12 },
};

const struct fl2000_mode *fl2000_hw_find_mode(int width, int height,
					      int refresh)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(fl2000_modes); i++) {
		const struct fl2000_mode *m = &fl2000_modes[i];

		if (m->width == width && m->height == height &&
		    m->refresh == refresh)
			return m;
	}
	return NULL;
}

/* fl2000_monitor_set_resolution(), bulk pipe, EOF = zero-length packet */
int fl2000_hw_set_mode(struct fl2000 *fl, const struct fl2000_mode *mode,
		       u32 wire_bpp)
{
	u32 h2 = mode->h_sync_2, v2 = mode->v_sync_2;
	u32 val;
	int ret;

	/* HDMI transmitter compliance tweaks from the reference driver */
	if (fl->ite_present) {
		if (mode->width == 640 && mode->height == 480) {
			h2 = 0x600091;
			v2 = 0x2420024;
		} else if (mode->width == 1280 && mode->height == 720) {
			v2 = 0x1A5001A;
		}
	}

	mutex_lock(&fl->hw_lock);

	ret = fl2000_reg_write_locked(fl, 0x802C, mode->pll);
	if (ret)
		goto out;
	ret = fl2000_reg_update_locked(fl, 0x8048, 0, BIT(15));
	if (ret)
		goto out;
	ret = fl2000_reg_read_locked(fl, 0x802C, &val);
	if (ret)
		goto out;
	if (val != mode->pll) {
		ret = -EIO;
		goto out;
	}

	/*
	 * The bit operations below mirror the reference driver (and the
	 * validated userspace harness) op for op, one bit per transfer, in
	 * the same order. Combining them into single register writes broke
	 * the chip's format latch: the CCS reset (bit 0) must be pulsed
	 * BEFORE the color format bits are set, never together with them.
	 */

	/* 0x803C: no BIA, no isoch interrupts/recovery, EOF type */
	{
		static const u8 clear_803c[] = { 22, 24, 19, 21, 13, 27, 28, 29 };
		int i;

		for (i = 0; i < ARRAY_SIZE(clear_803c); i++) {
			ret = fl2000_reg_update_locked(fl, 0x803C,
						       BIT(clear_803c[i]), 0);
			if (ret)
				goto out;
		}
		/*
		 * Bit 28 = EOF by ZLP. The alternative pending-bit mode
		 * (bit 29) was tested on hardware: the chip NAKs the whole
		 * bulk stream — do not use.
		 */
		ret = fl2000_reg_update_locked(fl, 0x803C, 0, BIT(28));
		if (ret)
			goto out;
	}

	/*
	 * 0x8004: clean format config, pulse the watermark clear, then
	 * select format. Bit 1 (frame_sync) survives the app reset, so
	 * clear it here too — with it set, scanout slaves to USB frame
	 * delivery and the refresh wobbles with host jitter (ITE loses
	 * video-stable); tested and rejected as a blink fix.
	 */
	{
		static const u8 clear_8004[] = { 28, 6, 31, 24, 25, 26, 27, 1 };
		int i;

		for (i = 0; i < ARRAY_SIZE(clear_8004); i++) {
			ret = fl2000_reg_update_locked(fl, 0x8004,
						       BIT(clear_8004[i]), 0);
			if (ret)
				goto out;
		}
		ret = fl2000_reg_update_locked(fl, 0x8004, 0, BIT(0));
		if (ret)
			goto out;
		if (wire_bpp == 2)
			ret = fl2000_reg_update_locked(fl, 0x8004, 0, BIT(6));
		else if (wire_bpp == 4)
			ret = fl2000_reg_update_locked(fl, 0x8004, 0, BIT(28));
		if (ret)
			goto out;
		ret = fl2000_reg_update_locked(fl, 0x8004, 0, BIT(7));
		if (ret)
			goto out;
	}

	ret = fl2000_reg_write_locked(fl, 0x8008, mode->h_sync_1);
	if (ret)
		goto out;
	ret = fl2000_reg_write_locked(fl, 0x800C, h2);
	if (ret)
		goto out;
	ret = fl2000_reg_write_locked(fl, 0x8010, mode->v_sync_1);
	if (ret)
		goto out;
	ret = fl2000_reg_write_locked(fl, 0x8014, v2);
	if (ret)
		goto out;

	/* clear isochronous config */
	ret = fl2000_reg_read_locked(fl, 0x801C, &val);
	if (ret)
		goto out;
	ret = fl2000_reg_write_locked(fl, 0x801C, val & 0xC000FFFF);
	if (ret)
		goto out;

	ret = fl2000_reg_update_locked(fl, 0x0070, 0, BIT(13));
	if (ret)
		goto out;
	ret = fl2000_reg_update_locked(fl, 0x8088, BIT(10), 0);

out:
	mutex_unlock(&fl->hw_lock);
	return ret;
}

/* fl2000_monitor_connect() prep: no U1/U2 while displaying */
int fl2000_hw_stream_prep(struct fl2000 *fl)
{
	int ret;

	mutex_lock(&fl->hw_lock);
	ret = fl2000_reg_update_locked(fl, 0x0078, BIT(17), 0);
	if (!ret)
		ret = fl2000_reg_update_locked(fl, 0x0070, 0,
					       BIT(19) | BIT(20));
	mutex_unlock(&fl->hw_lock);
	return ret;
}

const struct fl2000_mode *fl2000_hw_mode_table(int *count)
{
	*count = ARRAY_SIZE(fl2000_modes);
	return fl2000_modes;
}
