/* SPDX-License-Identifier: GPL-2.0 */
/*
 * fl2000drm - DRM driver for Fresco Logic FL2000DX USB display adapters
 *
 * Register protocol derived from Fresco Logic's GPL reference driver.
 */
#ifndef FL2000_DRM_H
#define FL2000_DRM_H

#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_simple_kms_helper.h>

struct fl2000_mode {
	u16 width;
	u16 height;
	u16 refresh;
	u32 h_sync_1;	/* reg 0x8008: width<<16 | htotal */
	u32 h_sync_2;	/* reg 0x800C: hsync<<16 | (hsync+hbp+1) */
	u32 v_sync_1;	/* reg 0x8010: height<<16 | vtotal */
	u32 v_sync_2;	/* reg 0x8014: low 16 = vsync+vbp+1 */
	u32 pll;	/* reg 0x802C, bulk transfer PLL */
	u8 vic;		/* CEA VIC for the AVI infoframe, 0 if none */
	u8 vsync_time;	/* vsync width in lines (for DRM mode timings) */
	u8 v_back_porch;
	/* isochronous transfer variant, from big_table_16bit_r0 */
	u32 iso_pll;		/* reg 0x802C, 0 = mode not ISO-capable */
	u32 iso_h_sync_1;	/* reg 0x8008 */
	u16 iso_pkt_size;	/* bytes per data packet at 60 Hz */
	u16 iso_packets;	/* packets per frame at 60 Hz */
};

struct fl2000 {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;

	struct usb_device *udev;
	struct usb_interface *intf;
	/* DMA-capable device (the host controller) for PRIME imports */
	struct device *dmadev;
	bool usb3;
	bool ite_present;

	/* serializes all EP0 register access and I2C sequences */
	struct mutex hw_lock;
	/* serializes logical IT66121 operations (DDC reads, mode setup) */
	struct mutex ite_lock;

	/*
	 * Triple-buffered wire-format frames: atomic updates convert into
	 * write_buf and publish it as ready_buf; the stream worker claims
	 * ready_buf as send_buf. Neither side ever blocks on the other.
	 */
	spinlock_t frame_swap_lock;
	void *write_buf;
	void *ready_buf;
	void *send_buf;
	bool frame_ready;
	size_t frame_len;
	const struct fl2000_mode *mode;
	u32 wire_bpp;			/* 2 = RGB565, 3 = RGB888 */
	u64 period_pll_ns;		/* PLL-formula period (approximate!) */
	u64 frame_period_ns;		/* working period, FLL-measured */
	atomic_t inflight;		/* URBs submitted, not yet completed */
	unsigned long last_complete;	/* jiffies of the last URB completion */
	unsigned long completions;	/* total completions (diagnostics) */
	int last_status;		/* last non-zero URB status seen */
	unsigned int recoveries;	/* consecutive stall recovery attempts */
	unsigned long stall_at;		/* last_complete of a detected stall, 0=idle */
	u64 next_frame_ns;		/* pacing absolute deadline chain */
	u32 pace_occ_target;		/* pre-submission backlog target */
	int pace_occ;			/* last pre-submission backlog */
	bool streaming;
	struct work_struct stream_work;

	/*
	 * Pipelined bulk transfers: a ring of URBs kept in flight so the
	 * device-side FIFO never starves on producer jitter. Bulk URBs on
	 * one pipe complete in submission order, so a counting semaphore
	 * plus round-robin slot reuse is race-free.
	 */
	/*
	 * Ring must exceed the largest frame's URB count (2560x1080@16bpp
	 * = 86) or submission is semaphore-throttled to the wire and the
	 * pacing servo can never hold surplus to own the delivery phase.
	 */
	struct fl2000_xfer {
		struct urb *urb;
		u8 *buf;	/* usb_alloc_coherent, pre-mapped for DMA */
		dma_addr_t dma;
	} xfers[128];
	struct semaphore xfer_sem;
	unsigned int xfer_ring;
	atomic_t stream_error;
	struct workqueue_struct *stream_wq;

	/*
	 * Isochronous path: interface 1, EP 0x02, one URB per frame with
	 * 128 data packets + empty packets covering the 8000/s bus
	 * intervals. The USB3 companion descriptor sets the bandwidth per
	 * altsetting; the chip consumes the same pixel pipeline as bulk.
	 */
	struct usb_host_endpoint *iso_ep;
	struct urb *iso_urb[2];
	u8 *iso_buf[2];
	dma_addr_t iso_dma[2];
	size_t iso_frame_len;
	unsigned int iso_packets;
	unsigned int iso_data_packets;
	unsigned int iso_pkt_size;
	unsigned int iso_pkt_count;
	unsigned int iso_eof_bit;
	bool iso_mframe_update;
	unsigned int iso_alt;
	unsigned int iso_ring;
	struct semaphore iso_free;
	struct work_struct iso_work;
	atomic_t iso_inflight;
	atomic_t iso_missed;
	bool iso;
	bool iso_failed;

	/*
	 * Interrupt EP (EP3 IN on interface 2): the chip signals VGA
	 * status events (monitor/EDID hotplug, frame drops, line-buffer
	 * overflow/underflow) via reg 0x8000. RE instrumentation.
	 */
	struct usb_interface *intr_intf;
	struct urb *intr_urb;
	u8 *intr_buf;
	dma_addr_t intr_dma;
	struct work_struct intr_work;
	atomic_t intr_count;
	u32 last_frame_cnt;

	/*
	 * Async connector status: userspace connector ioctls only ever read
	 * this cached state. All HPD/EDID I2C traffic happens in status_work
	 * — a slow or hung DDC (bus-hang aborts run 20s per block) must
	 * never run inside a probe ioctl or it blocks Xorg's whole event
	 * loop and the desktop appears frozen.
	 */
	struct work_struct status_work;
	atomic_t hpd_status;		/* last known: 1 = connected */
	int edid_attempts;		/* worker-only, reset on hotplug */
	/* protects edid_cache/edid_len/edid_valid */
	struct mutex edid_lock;
	u8 edid_cache[4 * 128];
	int edid_len;
	bool edid_valid;
};

#define to_fl2000(x) container_of(x, struct fl2000, drm)

#define FL2000_CHUNK_SIZE	(64 * 1024)

/* fl2000_hw.c */
int fl2000_reg_read(struct fl2000 *fl, u16 offset, u32 *val);
int fl2000_reg_read_try(struct fl2000 *fl, u16 offset, u32 *val);
int fl2000_reg_write(struct fl2000 *fl, u16 offset, u32 val);
int fl2000_i2c_read32(struct fl2000 *fl, u8 addr, u8 offset, u32 *val);
int fl2000_i2c_write32(struct fl2000 *fl, u8 addr, u8 offset, u32 val);
int fl2000_hw_reset(struct fl2000 *fl);
int fl2000_hw_dongle_init(struct fl2000 *fl);
int fl2000_hw_set_mode(struct fl2000 *fl, const struct fl2000_mode *mode,
		       u32 wire_bpp, bool iso);
int fl2000_hw_stream_prep(struct fl2000 *fl);
const struct fl2000_mode *fl2000_hw_find_mode(int width, int height,
					      int refresh);
const struct fl2000_mode *fl2000_hw_mode_table(int *count);

#define FL2000_NUM_XFERS	ARRAY_SIZE(((struct fl2000 *)0)->xfers)

/* fl2000_ite.c */
bool fl2000_ite_detect(struct fl2000 *fl);
int fl2000_ite_init(struct fl2000 *fl);
int fl2000_ite_hpd(struct fl2000 *fl);
int fl2000_ite_read_edid_block(void *ctx, u8 *buf, unsigned int block,
			       size_t len);
int fl2000_ite_enable_video(struct fl2000 *fl, const struct fl2000_mode *mode);
int fl2000_ite_video_off(struct fl2000 *fl);
int fl2000_ite_status(struct fl2000 *fl, u8 *sys_status);

#endif
