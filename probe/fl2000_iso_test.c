// SPDX-License-Identifier: GPL-2.0
/*
 * Userspace ISO bring-up harness for FL2000DX + IT66121 dongles.
 *
 * Same register bring-up as probe/fl2000_display.py (validated on hardware
 * over bulk), extended with the isochronous transport and live status
 * telemetry. Requires the fl2000drm kernel module to be UNLOADED and
 * blacklisted (it auto-claims the device on plug):
 *
 *   echo blacklist fl2000drm | sudo tee /etc/modprobe.d/fl2000-harness.conf
 *   sudo modprobe -r fl2000drm
 *
 * Build:
 *   gcc -O2 -Wall -o fl2000_iso_test fl2000_iso_test.c -lusb-1.0 -lm
 *
 * Run (root):
 *   sudo ./fl2000_iso_test --transport bulk                  # sanity: bars
 *   sudo ./fl2000_iso_test                                    # iso, kernel-like
 *   sudo ./fl2000_iso_test --pkts 128 --count 133            # no pad pkts
 *   sudo ./fl2000_iso_test --timing bulk                     # iso + CEA timing
 *   sudo ./fl2000_iso_test --recover                         # isoch rec bits
 *
 * Protocol from Fresco Logic's GPL reference driver (see notes/PROTOCOL.md).
 */
#define _GNU_SOURCE
#include <libusb.h>
#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define VID_FL2000	0x1D5C
#define PID_FL2000	0x2000

#define REQ_REG_READ	64
#define REQ_REG_WRITE	65
#define RT_READ		(LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | \
			 LIBUSB_RECIPIENT_DEVICE)
#define RT_WRITE	(LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | \
			 LIBUSB_RECIPIENT_DEVICE)

#define REG_I2C_CTRL	0x8020
#define REG_I2C_RDATA	0x8024
#define REG_I2C_WDATA	0x8028
#define I2C_ADDR_ITE	0x4C

/* 1920x1080@60 RGB565, bulk (CEA) and ISO table variants, plus a
 * CVT reduced-blanking variant (2080x1111 @ 138.33 MHz, 59.87 Hz) —
 * the anti-underflow candidate: -6.6% pixel-clock drain at scanout.
 */
#define MODE_W		1920
#define MODE_H		1080
#define MODE_FREQ	60
#define FRAME_LEN	(MODE_W * MODE_H * 2)
#define PKT_SIZE	(FRAME_LEN / 128)
#define DATA_PKTS	128
#define PLL_BULK	0x00596106
#define PLL_ISO		0x00776204
#define H1_BULK		0x7800898
#define H1_ISO		0x7800897
#define H2		0x2C00C1
#define V1		0x4380465
#define V2		0x2A5002A
/* horizontal reduced-blanking: htotal 2080 (hblank 160) + CEA vblank.
 * True CVT-RB (vtotal 1111, vblank 31) is impossible on this chip: the
 * low 16 bits of 0x8014 (vsync+vbp+1) must be >= ~42 or the scan engine
 * degrades to 1/3 speed (bisected on hardware). With CEA vblank the
 * constraint holds; htotal 2080 alone is proven working at full rate.
 * PLL: VCO 700 MHz (mult 70, presc 1, div 5) = 140.0 MHz → 59.96 Hz.
 */
#define PLL_RB		0x00466105
#define H1_RB		0x7800820
#define H2_RB		0x00200071
#define V1_RB		0x4380465
#define V2_RB		0x2A5002A
#define VIC_1080P	16
#define PIXCLK_10KHZ	14850
#define PIXCLK_RB_10KHZ	14000

/* VGA status register 0x8000 */
#define ST_VGA_ERROR	BIT(1)
#define ST_LBUF_HALT	BIT(2)
#define ST_TD_DROP	BIT(4)
#define ST_LBUF_OVER	BIT(8)
#define ST_LBUF_UNDER	BIT(9)
#define ST_FRAME_CNT(v)	(((v) >> 10) & 0xFFFF)

#ifndef BIT
#define BIT(n)		(1U << (n))
#endif

static libusb_device_handle *uh;

static void die(const char *fmt, ...) __attribute__((noreturn));
static void die(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
	exit(1);
}

static int reg_read(uint16_t off, uint32_t *val)
{
	unsigned char d[4];
	int ret;

	ret = libusb_control_transfer(uh, RT_READ, REQ_REG_READ, 0, off,
				      d, sizeof(d), 2000);
	if (ret != 4)
		return -EIO;
	*val = d[0] | (d[1] << 8) | ((uint32_t)d[2] << 16) |
	       ((uint32_t)d[3] << 24);
	return 0;
}

static int reg_write(uint16_t off, uint32_t val)
{
	unsigned char d[4];
	int ret;

	d[0] = val & 0xFF;
	d[1] = (val >> 8) & 0xFF;
	d[2] = (val >> 16) & 0xFF;
	d[3] = (val >> 24) & 0xFF;
	ret = libusb_control_transfer(uh, RT_WRITE, REQ_REG_WRITE, 0, off,
				      d, sizeof(d), 2000);
	return ret == 4 ? 0 : -EIO;
}

static uint32_t rr(uint16_t off)
{
	uint32_t v = 0;

	if (reg_read(off, &v))
		die("register 0x%04X read failed", off);
	return v;
}

static void rw(uint16_t off, uint32_t val)
{
	if (reg_write(off, val))
		die("register 0x%04X write failed", off);
}

static void bit_set(uint16_t off, int bit)
{
	rw(off, rr(off) | BIT(bit));
}

static void bit_clear(uint16_t off, int bit)
{
	rw(off, rr(off) & ~BIT(bit));
}

/*
 * One 4-byte I2C transaction on the FL2000 master. Bit 28 must be
 * written as 1 even though it reads back 0 (hardware quirk).
 */
static int i2c_op(uint8_t addr, uint8_t offset, bool read, uint32_t *val)
{
	uint32_t ctrl;
	int i;

	if (!read && reg_write(REG_I2C_WDATA, *val))
		return -EIO;
	if (reg_read(REG_I2C_CTRL, &ctrl))
		return -EIO;
	ctrl |= BIT(28);
	ctrl &= ~(BIT(31) | BIT(17) | BIT(16) | 0xFFFF);
	ctrl |= addr & 0x7F;
	ctrl |= (read ? 1 : 0) << 7;
	ctrl |= (offset & 0xFF) << 8;
	if (reg_write(REG_I2C_CTRL, ctrl))
		return -EIO;
	usleep(3000);
	for (i = 0; i < 100; i++) {
		if (reg_read(REG_I2C_CTRL, &ctrl))
			return -EIO;
		if (ctrl & BIT(31)) {
			if ((ctrl >> 24) & 0xF)
				return -EREMOTEIO;
			if (read && reg_read(REG_I2C_RDATA, val))
				return -EIO;
			return 0;
		}
		usleep(10000);
	}
	return -ETIMEDOUT;
}

/* ---- IT66121 byte access over the 4-byte I2C engine (RMW) ---- */

static uint8_t ite_read_byte(uint8_t off)
{
	uint32_t dw = 0;

	if (i2c_op(I2C_ADDR_ITE, off & ~3, true, &dw))
		die("ite read 0x%02X failed", off);
	return (dw >> ((off & 3) * 8)) & 0xFF;
}

static void ite_write_byte(uint8_t off, uint8_t val)
{
	uint32_t dw = 0;

	if (i2c_op(I2C_ADDR_ITE, off & ~3, true, &dw))
		die("ite rmw-read 0x%02X failed", off);
	dw = (dw & ~(0xFFU << ((off & 3) * 8))) |
	     ((uint32_t)val << ((off & 3) * 8));
	if (i2c_op(I2C_ADDR_ITE, off & ~3, false, &dw))
		die("ite write 0x%02X failed", off);
}

static void ite_write_dword(uint8_t off, uint32_t val)
{
	if (i2c_op(I2C_ADDR_ITE, off, false, &val))
		die("ite write32 0x%02X failed", off);
}

static void ite_write_masked(uint8_t off, uint8_t mask, uint8_t val)
{
	ite_write_byte(off, (ite_read_byte(off) & ~mask) | (val & mask));
}

static void ite_bit_set(uint8_t off, int bit)
{
	ite_write_byte(off, ite_read_byte(off) | BIT(bit));
}

static void ite_switch_bank(int bank)
{
	ite_write_byte(0x0F, bank & 1);
}

static void ite_reset(void)
{
	ite_bit_set(0x04, 5);			/* SW_RST */
	usleep(300000);
}

static void ite_power_up(void)
{
	static const struct { uint8_t reg, mask, val; } tbl[] = {
		{ 0x0F, 0x78, 0x38 },		/* PwrOn GRCLK */
		{ 0x05, 0x01, 0x00 },		/* PwrOn PCLK */
		{ 0x61, 0x20, 0x00 },		/* PwrOn DRV */
		{ 0x62, 0x44, 0x00 },		/* PwrOn XPLL */
		{ 0x64, 0x40, 0x00 },		/* PwrOn IPLL */
		{ 0x61, 0x10, 0x00 },		/* DRV_RST off */
		{ 0x62, 0x08, 0x08 },		/* XP_RESETB */
		{ 0x64, 0x04, 0x04 },		/* IP_RESETB */
		{ 0x6A, 0xFF, 0x70 },
		{ 0x66, 0xFF, 0x1F },
		{ 0x63, 0xFF, 0x38 },
		{ 0x0F, 0x78, 0x08 },		/* PwrOn IACLK */
	};
	size_t i;

	for (i = 0; i < sizeof(tbl) / sizeof(tbl[0]); i++) {
		if (tbl[i].mask == 0xFF)
			ite_write_byte(tbl[i].reg, tbl[i].val);
		else
			ite_write_masked(tbl[i].reg, tbl[i].mask, tbl[i].val);
	}
}

static void ite_av_mute(int mute)
{
	ite_switch_bank(0);
	ite_write_masked(0xC1, 0x01, mute);
	ite_write_byte(0xC6, 0x03);
}

static void ite_send_avi_infoframe(int vic)
{
	uint8_t db[13] = { 0 };
	uint8_t checksum;
	uint32_t dw;
	int sum, i;

	db[0] = (0 << 5) | (1 << 4);		/* RGB, active-format present */
	if (vic == 4 || vic == 16)			/* 16:9, ITU709 */
		db[1] = 8 | (2 << 4) | (2 << 6);
	else					/* 4:3, ITU601 */
		db[1] = 8 | (1 << 4) | (1 << 6);
	db[3] = vic;
	sum = 0x82 + 0x02 + 13;
	for (i = 0; i < 13; i++)
		sum += db[i];
	checksum = (0x100 - sum) & 0xFF;

	ite_switch_bank(1);
	dw = db[0] | (db[1] << 8) | (db[2] << 16) | ((uint32_t)db[3] << 24);
	ite_write_dword(0x58, dw);
	dw = db[4] | (checksum << 8) | (db[5] << 16) | ((uint32_t)db[6] << 24);
	ite_write_dword(0x5C, dw);
	dw = db[7] | (db[8] << 8) | (db[9] << 16) | ((uint32_t)db[10] << 24);
	ite_write_dword(0x60, dw);
	ite_write_dword(0x64, db[11] | (db[12] << 8));
	ite_switch_bank(0);
	ite_write_byte(0xCD, 0x03);		/* AVI infoframe: enable+repeat */
}

static void ite_enable_video(bool dvi)
{
	bool level_high = PIXCLK_10KHZ * 10000 > 80000000;
	uint8_t cur;

	ite_write_byte(0x04, 0x09);		/* VID_RST | HDCP_RST */
	cur = ite_read_byte(0x70);
	cur &= ~((3 << 6) | BIT(4) | BIT(3) | BIT(2) | BIT(5));
	ite_write_byte(0x70, cur | 0x01);	/* RGB, single edge, syncs */
	ite_write_masked(0x0F, 0x10, 0x10);	/* CSC bypass */
	cur = ite_read_byte(0x72);
	cur &= ~(3 | BIT(5) | BIT(7) | BIT(6));
	ite_write_byte(0x72, cur);
	ite_write_byte(0xC0, dvi ? 0 : 1);	/* DVI / HDMI mode */
	ite_write_byte(0x61, 0x10);		/* AFE_DRV_RST */
	if (level_high) {
		ite_write_masked(0x62, 0x90, 0x80);
		ite_write_masked(0x64, 0x89, 0x80);
		ite_write_masked(0x68, 0x10, 0x80);
	} else {
		ite_write_masked(0x62, 0x90, 0x10);
		ite_write_masked(0x64, 0x89, 0x09);
		ite_write_masked(0x68, 0x10, 0x10);
	}
	ite_write_masked(0x04, 0x28, 0x00);	/* release REF_RST | VID_RST */
	ite_write_byte(0x61, 0x00);
	ite_write_byte(0x04, 0x01);		/* keep only HDCP_RST */
	ite_switch_bank(0);
	ite_write_byte(0x61, 0x00);		/* fire AFE */
}

static uint8_t ite_sys_status(void)
{
	return ite_read_byte(0x0E);
}

/* ---- test pattern ---- */

static const uint16_t bar_colors[8] = {
	0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x0000
};

static uint8_t frame_row[MODE_W * 2];
static uint8_t frame_buf[FRAME_LEN];

static void pattern_build(int frame_idx)
{
	uint8_t le16[2];
	uint32_t shift, rowlen = sizeof(frame_row);
	int b, x;

	for (b = 0; b < 8; b++) {
		le16[0] = bar_colors[b] & 0xFF;
		le16[1] = bar_colors[b] >> 8;
		for (x = 0; x < MODE_W / 8; x++) {
			frame_row[b * (MODE_W / 8) + x] = le16[0];
			frame_row[b * (MODE_W / 8) * 2 + x] = 0;	/* overwritten */
		}
	}
	/* rebuild row properly (little-endian u16 per pixel) */
	for (x = 0; x < MODE_W; x++) {
		uint16_t c = bar_colors[(x * 8) / MODE_W];

		frame_row[x * 2] = c & 0xFF;
		frame_row[x * 2 + 1] = c >> 8;
	}

	shift = (frame_idx * 16) % rowlen;
	for (x = 0; x < MODE_H; x++) {
		uint32_t src = shift;
		uint8_t *dst = frame_buf + (size_t)x * rowlen;

		memcpy(dst, frame_row + src, rowlen - src);
		memcpy(dst + (rowlen - src), frame_row, src);
	}

	/* 64-bit word swap: adjacent 32-bit halves reversed */
	for (x = 0; x < FRAME_LEN / 8; x++) {
		uint32_t *p32 = (uint32_t *)(frame_buf + (size_t)x * 8);
		uint32_t tmp = p32[0];

		p32[0] = p32[1];
		p32[1] = tmp;
	}
}

/* ---- status telemetry ---- */

static uint16_t last_cnt;
static int have_last_cnt;
static uint32_t fb_last_val;
static int fb_count;

static void status_print(bool force)
{
	static uint64_t last_ns;
	static int n;
	uint64_t now = 0;
	struct timespec ts;
	uint32_t st;
	uint8_t ite;
	int16_t dc = -1;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	now = (uint64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
	if (!force && now - last_ns < 1000000000)
		return;
	last_ns = now;

	st = rr(0x8000);
	ite = ite_sys_status();
	if (have_last_cnt)
		dc = (int16_t)(ST_FRAME_CNT(st) - last_cnt);
	last_cnt = ST_FRAME_CNT(st);
	have_last_cnt = 1;

	printf("[%2d.%01d] st=0x%08X frame=%5u (%+4d) eng=0x%08X%s%s%s%s%s | ite=0x%02X%s%s%s%s\n",
	       n / 10, n % 10, st, ST_FRAME_CNT(st), dc, rr(0x8080),
	       st & ST_LBUF_UNDER ? " UNDER" : "",
	       st & ST_LBUF_OVER ? " OVER" : "",
	       st & ST_LBUF_HALT ? " HALT" : "",
	       st & ST_TD_DROP ? " TD-DROP" : "",
	       st & BIT(3) ? " ISO-ACK" : "",
	       ite,
	       ite & 0x40 ? " HPD" : "",
	       ite & 0x20 ? " RxSense" : "",
	       ite & 0x10 ? " VID-STABLE" : "",
	       fb_count ? "" : "");
	if (fb_count) {
		printf("        fb=%u samples, last=0x%08X (rate %.6f/int)\n",
		       fb_count, fb_last_val,
		       (double)fb_last_val / 8192.0);
	}
	fflush(stdout);

	/* clear the sticky lbuf bits by writing them back */
	if (st & (ST_LBUF_OVER | ST_LBUF_UNDER))
		rw(0x8000, st);
	n++;
}

/* ---- ISO transport ---- */

#define MAX_ISO_XFERS	2

struct iso_xfer {
	struct libusb_transfer *t;
	uint8_t *buf;
	int busy;
	int missed;
	int status;
};

static struct iso_xfer iso_x[MAX_ISO_XFERS];
static int iso_nx;
static int iso_pkts = 133;
static int iso_inflight = 2;
static int shortlast;		/* trim N bytes off the last data packet */
static int bigdescs;		/* one/few large descriptors per URB */

static void iso_complete(struct libusb_transfer *t)
{
	struct iso_xfer *x = t->user_data;
	int i;

	x->status = t->status;
	x->missed = 0;
	if (!t->status)
		for (i = 0; i < t->num_iso_packets; i++)
			if (t->iso_packet_desc[i].status)
				x->missed++;
	x->busy = 0;
}

static void iso_submit(struct iso_xfer *x, int frame_idx)
{
	int i, ret;

	pattern_build(frame_idx);
	memcpy(x->buf, frame_buf, FRAME_LEN);
	if (bigdescs) {
		/* one URB = few large descriptors = few TDs on the wire:
		 * the xHCI P-bit clears only at each TD's last packet,
		 * giving the chip a single EOF per frame
		 */
		for (i = 0; i < iso_pkts; i++)
			x->t->iso_packet_desc[i].length = FRAME_LEN / iso_pkts;
	} else {
		for (i = 0; i < iso_pkts; i++) {
			int len = i < DATA_PKTS ? PKT_SIZE : 0;

			/* short-packet EOF variant: last data packet trimmed */
			if (shortlast && i == DATA_PKTS - 1)
				len -= shortlast;
			x->t->iso_packet_desc[i].length = len;
		}
	}
	for (i = 0; i < iso_pkts; i++) {
		x->t->iso_packet_desc[i].actual_length = 0;
		x->t->iso_packet_desc[i].status = 0;
	}
	x->busy = 1;
	x->status = 0;
	x->missed = 0;
	ret = libusb_submit_transfer(x->t);
	if (ret)
		die("iso submit failed: %s (%d)", libusb_error_name(ret), ret);
}

/* ---- feedback endpoint polling (EP 0x82 ISO IN, adaptive-sink) ---- */

static struct libusb_transfer *fb_t;
static uint8_t fb_buf[8 * 8];
static volatile int fb_seq;

static void fb_complete(struct libusb_transfer *t)
{
	int i;

	for (i = 0; i < t->num_iso_packets; i++) {
		unsigned char *p;

		if (t->iso_packet_desc[i].status)
			continue;
		p = libusb_get_iso_packet_buffer_simple(t, i);
		if (p) {
			fb_last_val = p[0] | (p[1] << 8) | ((uint32_t)p[2] << 16);
			fb_count++;
		}
	}
	fb_seq++;
	if (libusb_submit_transfer(t))
		t->status = LIBUSB_TRANSFER_CANCELLED;
}

static void feedback_start(void)
{
	fb_t = libusb_alloc_transfer(8);
	if (!fb_t)
		die("fb alloc failed");
	libusb_fill_iso_transfer(fb_t, uh, 0x82, fb_buf, sizeof(fb_buf),
				 8, fb_complete, NULL, 5000);
	libusb_set_iso_packet_lengths(fb_t, 8);
	if (libusb_submit_transfer(fb_t))
		die("feedback submit failed");
}

static void dump_if1_endpoints(int alt)
{
	struct libusb_config_descriptor *cfg = NULL;
	const struct libusb_interface_descriptor *d = NULL;
	const struct libusb_interface *intf;
	int i, j;

	if (libusb_get_active_config_descriptor(libusb_get_device(uh), &cfg))
		return;
	if (cfg->bNumInterfaces > 1) {
		intf = &cfg->interface[1];
		for (j = 0; j < intf->num_altsetting; j++)
			if (intf->altsetting[j].bAlternateSetting == alt)
				d = &intf->altsetting[j];
	}
	if (!d) {
		printf("IF1 alt %d: not found\n", alt);
		goto out;
	}
	printf("IF1 alt %d: %d endpoints\n", alt, d->bNumEndpoints);
	for (i = 0; i < d->bNumEndpoints; i++) {
		const struct libusb_endpoint_descriptor *e = &d->endpoint[i];
		const unsigned char *x = e->extra;
		int len = e->extra_length;

		printf("  EP 0x%02X attr=0x%02X maxp=%u interval=%u",
		       e->bEndpointAddress, e->bmAttributes, e->wMaxPacketSize,
		       e->bInterval);
		/* parse the SuperSpeed companion out of the extra blob */
		while (len >= 4 && x[0] >= 4 && len >= (int)x[0]) {
			if (x[1] == 0x30) {	/* SS endpoint companion */
				printf(" ss(maxburst=%u attr=0x%02X "
				       "bytesPerInterval=%u)",
				       x[2], x[3],
				       x[4] | ((unsigned)x[5] << 8));
				break;
			}
			len -= x[0];
			x += x[0];
		}
		printf("\n");
	}
out:
	libusb_free_config_descriptor(cfg);
}

/* ---- mid-stream full register dump (--dump) ---- */

static bool dump_enabled;
static bool dump_done;

static void dump_all_regs(const char *when)
{
	static const uint16_t regs[] = {
		0x0068, 0x006C, 0x0070, 0x0074, 0x0078, 0x007C,
		0x8000, 0x8004, 0x8008, 0x800C, 0x8010, 0x8014,
		0x8018, 0x801C, 0x8020, 0x8024, 0x8028, 0x802C,
		0x8030, 0x8034, 0x8038, 0x803C, 0x8040, 0x8044,
		0x8048, 0x804C, 0x8050, 0x8054, 0x8058, 0x805C,
		0x8064, 0x806C, 0x8070, 0x8074, 0x8078, 0x807C,
		0x8080, 0x8084, 0x8088, 0x808C,
	};
	size_t k;

	printf("---- register dump (%s) ----\n", when);
	for (k = 0; k < sizeof(regs) / sizeof(regs[0]); k++)
		printf("[0x%04X] = 0x%08X\n", regs[k], rr(regs[k]));
	fflush(stdout);
	dump_done = true;
}

/* ---- main ---- */

static void usage(const char *p)
{
	fprintf(stderr,
"usage: %s [options]\n"
"  --transport bulk|iso   (iso)   endpoint transport\n"
"  --timing iso|bulk     (iso)   scanout PLL + 0x8008 pair\n"
"  --eof 28|29           (29)    0x803C EOF type bit\n"
"  --count N             (133)   0x801C packets-per-frame count, 0=clear\n"
"  --idle N              (0x120) 0x801C idle interval field\n"
"  --pkts N              (133)   packets per URB (>=128, rest are zero-len)\n"
"  --inflight 1|2       (2)     URBs in flight (1 enables pacing)\n"
"  --nopace                       with --inflight 1: submit ASAP, no delay\n"
"  --mframe                       set 0x801C bit 30 (MFRAME_CNT_UPDATE)\n"
"  --recover                       set 0x803C isoch bits 13,19,21\n"
"  --alt N              (4)      IF1 iso altsetting\n"
"  --watermark N                 write 0x8030 lbuf watermark\n"
"  --seconds N          (12)     streaming duration\n"
"  --static                       no scrolling\n"
"  --dump                         bring up, print regs, don't stream\n"
"  --dvi                          DVI output instead of HDMI\n",
		p);
	exit(2);
}

int main(int argc, char **argv)
{
	const char *transport = "iso", *timing = "iso";
	int eof_bit = 29, idle = 0x120, alt = 4, seconds = 12;
	int count = 133, watermark = -1;
	int htotal_ov = 0;		/* 0 = table/CEA/iso value */
	uint32_t pll_ov = 0;		/* 0 = don't override */
	bool mframe = false, recover = false, static_pat = false;
	bool dump = false, dvi = false, pace = true, feedback = false, rb = false;
	uint32_t poke_val[32];
	uint16_t poke_reg[32];
	int n_pokes = 0;
	uint32_t pll, h1, h2, val;
	struct timespec ts;
	uint64_t deadline, next_submit = 0;
	int frame_idx = 0, sent = 0, i, ret;
	uint32_t dw;

	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--transport") && i + 1 < argc)
			transport = argv[++i];
		else if (!strcmp(argv[i], "--timing") && i + 1 < argc)
			timing = argv[++i];
		else if (!strcmp(argv[i], "--eof") && i + 1 < argc)
			eof_bit = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--count") && i + 1 < argc)
			count = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--idle") && i + 1 < argc)
			idle = strtoul(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--pkts") && i + 1 < argc)
			iso_pkts = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--shortlast") && i + 1 < argc)
			shortlast = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--bigdescs") && i + 1 < argc) {
			bigdescs = 1;
			iso_pkts = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i], "--inflight") && i + 1 < argc)
			iso_inflight = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--nopace"))
			pace = false;
		else if (!strcmp(argv[i], "--mframe"))
			mframe = true;
		else if (!strcmp(argv[i], "--recover"))
			recover = true;
		else if (!strcmp(argv[i], "--alt") && i + 1 < argc)
			alt = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--watermark") && i + 1 < argc)
			watermark = strtoul(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--seconds") && i + 1 < argc)
			seconds = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--static"))
			static_pat = true;
		else if (!strcmp(argv[i], "--dump"))
			dump = true;
		else if (!strcmp(argv[i], "--poke") && i + 1 < argc) {
			char *s = argv[++i];

			if (n_pokes >= 32)
				die("too many --poke");
			poke_reg[n_pokes] = strtoul(s, &s, 0);
			if (*s != ':')
				die("--poke REG:VAL");
			poke_val[n_pokes++] = strtoul(s + 1, NULL, 0);
		}
		else if (!strcmp(argv[i], "--feedback"))
			feedback = true;
		else if (!strcmp(argv[i], "--rb"))
			rb = true;
		else if (!strcmp(argv[i], "--htotal") && i + 1 < argc)
			htotal_ov = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--pll") && i + 1 < argc)
			pll_ov = strtoul(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--dvi"))
			dvi = true;
		else
			usage(argv[0]);
	}

	if (bigdescs) {
		if (iso_pkts < 1 || FRAME_LEN % iso_pkts)
			die("--bigdescs must divide %d", FRAME_LEN);
	} else if (iso_pkts < DATA_PKTS || iso_pkts > 1024) {
		die("--pkts must be >= %d", DATA_PKTS);
	}
	if (iso_inflight < 1 || iso_inflight > MAX_ISO_XFERS)
		die("--inflight must be 1..%d", MAX_ISO_XFERS);

	ret = libusb_init(NULL);
	if (ret)
		die("libusb_init: %s", libusb_error_name(ret));
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_ERROR);

	uh = libusb_open_device_with_vid_pid(NULL, VID_FL2000, PID_FL2000);
	if (!uh)
		die("FL2000 not found");
	printf("device open (1080p60 RGB565, transport=%s timing=%s eof=%d "
	       "count=%d pkts=%d inflight=%d mframe=%d recover=%d%s%s)\n",
	       transport, timing, eof_bit, count, iso_pkts, iso_inflight,
	       mframe, recover, pace ? "" : " nopace",
	       static_pat ? " static" : "");

	/* IT66121 sanity + bring-up (fl2000_dongle_card_initialize order) */
	dw = 0;
	if (i2c_op(I2C_ADDR_ITE, 0, true, &dw) || (dw & 0xFFFF) != 0x4954)
		die("IT66121 not responding");
	bit_set(0x8048, 15);			/* dongle reset */
	ite_reset();
	ite_power_up();
	printf("ITE up: HPD=%d RxSense=%d\n",
	       !!(ite_sys_status() & 0x40), !!(ite_sys_status() & 0x20));

	/* monitor-connect prep (fl2000_monitor_connect) */
	bit_clear(0x0078, 17);
	bit_set(0x0070, 19);
	bit_set(0x0070, 20);

	/* ---- fl2000_monitor_set_resolution, variant-driven ---- */
	if (rb) {
		pll = PLL_RB;
		h1 = H1_RB;
		h2 = H2_RB;
	} else {
		pll = !strcmp(timing, "bulk") ? PLL_BULK : PLL_ISO;
		h1 = !strcmp(timing, "bulk") ? H1_BULK : H1_ISO;
		h2 = H2;
	}
	if (htotal_ov > 0) {
		uint32_t hsync_start = MODE_W + 48;	/* RB hfront */

		h1 = (MODE_W << 16) | htotal_ov;
		h2 = (32 << 16) | (htotal_ov - hsync_start + 1);
		printf("htotal override %d (hblank %d): 0x8008=0x%08X "
		       "0x800C=0x%08X\n", htotal_ov, htotal_ov - MODE_W,
		       h1, h2);
	}
	if (pll_ov)
		pll = pll_ov;

	rw(0x802C, pll);
	if (!strcmp(transport, "iso")) {
		printf("waiting 1s for PLL stabilization...\n");
		sleep(1);
	}
	bit_set(0x8048, 15);			/* app reset, self-clearing */
	if (rr(0x802C) != pll)
		die("PLL readback mismatch");

	/* clear_803c[] = 22,24,19,21,13,27,28,29, one bit per transfer */
	{
		static const int clear_bits[] = { 22, 24, 19, 21, 13, 27, 28, 29 };
		size_t k;

		for (k = 0; k < sizeof(clear_bits) / sizeof(clear_bits[0]); k++)
			bit_clear(0x803C, clear_bits[k]);
		if (recover) {
			bit_set(0x803C, 13);	/* isoch feedback interrupt */
			bit_set(0x803C, 19);	/* isoch auto recover */
			bit_set(0x803C, 21);	/* isoch auto recover */
		}
		bit_set(0x803C, eof_bit);	/* EOF type */
	}

	/* 0x8004: clean format config, pulse CCS, select 565 */
	{
		static const int clear_bits[] = { 28, 6, 31, 24, 25, 26, 27, 1, 13 };
		size_t k;

		for (k = 0; k < sizeof(clear_bits) / sizeof(clear_bits[0]); k++)
			bit_clear(0x8004, clear_bits[k]);
		bit_set(0x8004, 0);
		bit_set(0x8004, 6);		/* RGB565 */
		bit_set(0x8004, 7);		/* external DAC (→ ITE) */
	}

	/* timing registers, one register per reference sequence */
	{
		struct { uint16_t off; uint32_t val; } regs[] = {
			{ 0x8008, h1 }, { 0x800C, h2 },
			{ 0x8010, rb ? V1_RB : V1 }, { 0x8014, rb ? V2_RB : V2 },
		};
		size_t k;

		for (k = 0; k < sizeof(regs) / sizeof(regs[0]); k++) {
			rw(regs[k].off, regs[k].val);
			if (rr(regs[k].off) != regs[k].val)
				die("timing reg 0x%04X readback mismatch",
				   regs[k].off);
		}
	}

	/* 0x801C: iso packet count + idle, or clear for bulk */
	val = rr(0x801C);
	if (!strcmp(transport, "iso") && count > 0) {
		val = (val & 0xC0000000) | ((uint32_t)count << 16) | idle;
		if (mframe)
			val |= BIT(30);
	} else {
		val &= 0xC000FFFF;
	}
	rw(0x801C, val);

	bit_set(0x0070, 13);
	bit_clear(0x8088, 10);
	if (watermark >= 0)
		rw(0x8030, (uint32_t)watermark);
	for (i = 0; i < n_pokes; i++) {
		printf("poke [0x%04X] = 0x%08X\n", poke_reg[i], poke_val[i]);
		rw(poke_reg[i], poke_val[i]);
	}

	printf("mode regs: 0x802C=0x%08X 0x8008=0x%08X 0x8004=0x%08X "
	       "0x803C=0x%08X 0x801C=0x%08X 0x8030=0x%08X\n",
	       rr(0x802C), rr(0x8008), rr(0x8004), rr(0x803C),
	       rr(0x801C), rr(0x8030));

	if (dump) {
		dump_enabled = true;
		dump_all_regs("pre-stream");
		dump_done = false;
	}

	ite_av_mute(1);
	ite_send_avi_infoframe(VIC_1080P);
	ite_enable_video(dvi);
	ite_av_mute(0);
	printf("IT66121 video output enabled (%s)\n", dvi ? "DVI" : "HDMI");

	clock_gettime(CLOCK_MONOTONIC, &ts);
	deadline = (uint64_t)ts.tv_sec * 1000000000 + ts.tv_nsec +
		   (uint64_t)seconds * 1000000000;

	if (!strcmp(transport, "bulk")) {
		int xfer;

		if (libusb_claim_interface(uh, 0))
			die("claim IF0 failed");
		if (libusb_set_interface_alt_setting(uh, 0, 1))
			die("IF0 alt 1 failed");
		printf("bulk streaming (ZLP end-of-frame)...\n");

		while (1) {
			clock_gettime(CLOCK_MONOTONIC, &ts);
			if ((uint64_t)ts.tv_sec * 1000000000 + ts.tv_nsec >
			    deadline)
				break;
			pattern_build(static_pat ? 0 : frame_idx++);
			xfer = libusb_bulk_transfer(uh, 0x01, frame_buf,
						    FRAME_LEN, &i, 5000);
			if (xfer)
				die("bulk write: %s", libusb_error_name(xfer));
			xfer = libusb_bulk_transfer(uh, 0x01, frame_buf, 0,
						    &i, 5000);
			if (xfer && xfer != LIBUSB_ERROR_PIPE)
				die("zlp: %s", libusb_error_name(xfer));
			sent++;
			status_print(false);
			if (dump_enabled && !dump_done && sent >= 180)
				dump_all_regs("mid-stream bulk");
		}
		libusb_release_interface(uh, 0);
	} else {
		if (libusb_claim_interface(uh, 1))
			die("claim IF1 failed");
		if (libusb_set_interface_alt_setting(uh, 1, alt))
			die("IF1 alt %d failed", alt);
		dump_if1_endpoints(alt);
		if (feedback)
			feedback_start();
		printf("iso streaming: alt %d, %d packets (%d x %d B + %d empty), "
		       "%d in flight%s...\n",
		       alt, iso_pkts, DATA_PKTS, PKT_SIZE,
		       iso_pkts - DATA_PKTS, iso_inflight,
		       pace ? ", paced" : "");

		iso_nx = iso_inflight;
		for (i = 0; i < iso_nx; i++) {
			iso_x[i].buf = malloc(FRAME_LEN);
			iso_x[i].t = libusb_alloc_transfer(iso_pkts);
			if (!iso_x[i].buf || !iso_x[i].t)
				die("oom");
			libusb_fill_iso_transfer(iso_x[i].t, uh, 0x02,
						 iso_x[i].buf, FRAME_LEN,
						 iso_pkts, iso_complete,
						 &iso_x[i], 5000);
			/* offsets auto-accumulate; zero-len pad needs none */
		}

		while (1) {
			uint64_t now;

			clock_gettime(CLOCK_MONOTONIC, &ts);
			now = (uint64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
			if (now > deadline)
				break;

			for (i = 0; i < iso_nx; i++) {
				if (iso_x[i].busy)
					continue;
				if (iso_inflight == 1 && pace) {
					uint64_t period =
						(uint64_t)count * 125000;

					if (next_submit && now < next_submit)
						usleep((next_submit - now) /
						       1000);
					clock_gettime(CLOCK_MONOTONIC, &ts);
					next_submit =
						(uint64_t)ts.tv_sec * 1000000000 +
						ts.tv_nsec + period;
				}
				iso_submit(&iso_x[i],
					   static_pat ? 0 : frame_idx++);
				sent++;
			}

			/* completions run inside handle_events */
			{
				struct timeval tv = { 0, 20000 };

				libusb_handle_events_timeout(NULL, &tv);
			}
			status_print(false);
			if (dump_enabled && !dump_done && sent >= 180)
				dump_all_regs("mid-stream iso");
		}

		/* drain and free: never free an active transfer */
		for (i = 0; i < iso_nx; i++) {
			int tries;

			for (tries = 0; tries < 100 && iso_x[i].busy; tries++) {
				struct timeval tv = { 0, 20000 };

				libusb_handle_events_timeout(NULL, &tv);
			}
			if (iso_x[i].busy) {
				libusb_cancel_transfer(iso_x[i].t);
				for (tries = 0; tries < 100 && iso_x[i].busy;
				     tries++) {
					struct timeval tv = { 0, 20000 };

					libusb_handle_events_timeout(NULL,
								     &tv);
				}
			}
			libusb_free_transfer(iso_x[i].t);
			free(iso_x[i].buf);
		}
		libusb_release_interface(uh, 1);
	}

	printf("done: %d frames in %d s (%.1f fps)\n",
	       sent, seconds, (double)sent / seconds);
	status_print(true);

	libusb_close(uh);
	libusb_exit(NULL);
	return 0;
}
