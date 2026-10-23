# FL2000DX USB protocol notes

Extracted from Fresco Logic's official (GPL) reference driver in
`reference/FL2000-official`. Register semantics come from that source; nothing
here is from the datasheet (there is no public one).

## Device

- USB ID `1d5c:2000`, USB 3.0 SuperSpeed device (enumerates at High Speed on
  USB2 ports; Fresco says video streaming is unsupported at USB2 speeds).
- Interfaces: IF0 = control/streaming (bulk EP), IF1/IF2 = other AV interfaces,
  IF3 = fake mass-storage "driver CD".
- No framebuffer on chip: host streams every frame over USB bulk. 1080p60@24bpp
  = 373 MB/s, hence USB3 requirement for anything real.

## Register (MMIO) access over EP0

All chip registers are 32-bit, addressed by byte offset, accessed with vendor
control transfers on EP0:

| op    | bmRequestType | bRequest | wValue | wIndex     | data       |
|-------|---------------|----------|--------|------------|------------|
| read  | 0xC0          | 64       | 0      | reg offset | 4 bytes LE |
| write | 0x40          | 65       | 0      | reg offset | 4 bytes LE |

Timeout used by reference driver: 2000 ms.

Register blocks seen in the reference driver: `0x0070`, `0x0078` (USB-side
control), `0x8000`–`0x8088` (VGA/video block).

- `0x8048` bit 15: "app reset", self-clearing (dongle soft reset).
- `0x0070` bit 13: set during monitor-connect handling; bit 20: "wakeup"
  (see `fl2000_dev.c`); bit 24-25: interrupt/wakeup related.

## I2C master (regs 0x8020/0x8024/0x8028)

The FL2000 has an I2C master used for DDC (monitor EDID @ 0x50) and for the
HDMI transmitter chip on HDMI dongles. Transactions move 4 bytes at a time.

`0x8020` — control/status, bitfields:

| bits  | field          | notes                                    |
|-------|----------------|------------------------------------------|
| 6:0   | Addr           | 7-bit I2C address                        |
| 7     | RW             | 1 = read, 0 = write                      |
| 15:8  | offset         | register offset within I2C target        |
| 16    | IsSpiOperation | 1 = SPI flash op, 0 = I2C                |
| 17    | SpiEraseEnable |                                          |
| 23:18 | reserved       |                                          |
| 27:24 | DataStatus     | per-byte status, 0 = ok, 1 = failed      |
| 28    | detect enable  | set at dongle init; **always reads 0 but |
|       |                | must be written back as 1** (hw quirk)   |
| 29    | reserved       |                                          |
| 30    | detect enable  | set at dongle init (see below)           |
| 31    | OpStatus       | 1 = done. Write 0 to start next op.      |

**Dongle init (after app reset): set bits 28 and 30 — VGA dongles ONLY.**
The reference driver sets both during card initialize (monitor-detect /
EDID-detect machinery; exact field names unconfirmed). On VGA-only
dongles (no IT66121) the internal VGA DAC never outputs sync without
them — black screen, and the bulk pipe eventually stalls (-ETIMEDOUT)
because scanout never starts consuming. Found and validated on hardware
by @ftoledo (issue #1, stable 1440x900 on a VGA-only dongle).
Do NOT set them on IT66121 dongles: observed on hardware to corrupt the
0x8004 format latch (readback grows phantom bits, e.g. 0xC000C44C where
a healthy run reads 0x00000000) and the bulk pipe never consumes at
1920x1080 → NAK-forever, -ETIMEDOUT after ~50 retry frames.

`0x8024` — read data (4 bytes, LE; byte at `offset+0` = bits 7:0).
`0x8028` — write data (4 bytes, LE).

Sequence (read): read 0x8020 → OR 0x10000000 → set Addr/RW=1/offset, clear
OpStatus/IsSpi/SpiErase → write 0x8020 → wait ≥3 ms → poll 0x8020 until bit 31
set (10 × 10 ms) → read data from 0x8024.

Sequence (write): write data dword to 0x8028 first, then same control dance
with RW=0.

## HDMI dongles: ITE IT66121 transmitter

- Sits on the FL2000 I2C bus at address **0x4C**.
- Detect: i2c read32 @0x4C offset 0 → low 16 bits vendor `0x4954` ("IT"),
  bits 27:16 & 0xFFF device `0x612`.
- After detection the reference driver does: dongle reset (0x8048 bit15),
  IT66121 reset + power-up register table (`fl2000_hdmi.c`), then reads EDID
  **through the IT66121's DDC master** (regs 0x10–0x17 on the ITE chip), not
  via direct 0x50 — on HDMI dongles the connector's DDC lines go to the ITE
  chip.
- VGA dongles instead: EDID read directly at i2c 0x50 (`fl2000_monitor.c`,
  `read_edid_dsub`), 128 bytes in 4-byte chunks; header check
  `00 FF FF FF FF FF FF 00`.

## Mode set (FL2000 side) — VERIFIED WORKING

Sequence from `fl2000_monitor_set_resolution()`, reproduced in
`probe/fl2000_display.py`:

1. `0x802C` ← PLL value from mode table; then app reset (`0x8048` bit 15);
   verify PLL readback.
2. `0x803C`: clear bits 22 (BIA), 24, 19, 21, 13 (isoch stuff), 27/28/29
   (EOF type); then set bit 28 = EOF-by-zero-length-packet (bit 29 would be
   "pending bit" mode).
3. `0x8004`: clear 28, 6, 31, 24 (compression), 25 (8-bit), 26 (palette),
   27 (first-byte mask); set bit 0 (VGA CCS reset, self-clears), bit 6
   (RGB565; bit 31 = RGB555; bit 25 = 8-bit), **bit 7 = external DAC enable —
   this routes pixels to the IT66121**. bit 24 = compression enable (needed
   for USB2 at 800x600+).
4. Timing regs (values straight from big-table): `0x8008`=h_sync_reg_1
   (width<<16|htotal), `0x800C`=h_sync_reg_2 (hsync<<16|(hsync+bp+1)),
   `0x8010`=v_sync_reg_1, `0x8014`=v_sync_reg_2. HDMI dongles: apply
   `fl2000_hdmi_compliance_tweak` overrides for 640x480@60 and 1280x720@60.
5. `0x801C` &= 0xC000FFFF (clear iso config), set `0x0070` bit 13,
   clear `0x8088` bit 10.

Mode table rows (16-bit R0, `fl2000_big_table.c`):

| mode | h_sync_1 | h_sync_2 | v_sync_1 | v_sync_2 | bulk PLL |
|------|----------|----------|----------|----------|----------|
| 640x480@60 | 0x2800320 | 0x600089 → 0x600091 (hdmi tweak) | 0x1E0020D | 0x1C2001C → 0x2420024 (tweak) | 0x3F6119 |
| 800x600@60 | 0x3200420 | 0x8000D9 | 0x2580274 | 0x1C4001C | 0x80102 |

(0x3F6119 is also the chip's power-on default PLL.)

## Frame streaming — VERIFIED WORKING

- USB: interface 0, **altsetting 1**, bulk EP 0x01 OUT (alt 0 has no usable
  EPs; this matches the reference driver's ≥5.x kernel workaround).
- One frame = one bulk transfer of raw pixels (RGB565 LE, rows top to
  bottom, no headers), followed by a **zero-length packet** as EOF marker.
- The device flow-controls bulk to the scanout rate: streaming 640x480@60
  RGB565 over a USB2 link sustains exactly 60.0 fps (36.9 MB/s).
- Before streaming: clear `0x0078` bit 17, set `0x0070` bits 19+20 (reject
  U1/U2 — from `fl2000_monitor_connect`).

## IT66121 video output — VERIFIED WORKING

From `fl2000_hdmi_set_display_mode()` (reproduced in fl2000_display.py):
av_mute(1) → AVI infoframe (bank 1 regs 0x58..0x65, ctrl 0xCD=0x03) →
enable video (SW_RST=0x09; input mode 0x70 = RGB separate-sync +bit0;
CSC bypass 0x72; 0xC0 = HDMI(1)/DVI(0); AFE setup by pixel clock band
(>80 MHz = high swing); SW_RST=0x01; fire AFE 0x61=0) → av_mute(0).
Status: reg 0x0E bit 4 = VIDEO_STABLE goes high while FL2000 streams.

## CRITICAL: bulk stream byte order (the pixel_swap quirk)

The FL2000 consumes the bulk stream as 64-bit words with the two 32-bit
halves **reversed**. Every frame must have adjacent dword pairs swapped
before transmission (`buf64[i] = rol64(buf64[i], 32)` over the whole
frame). The reference driver does this in `pixel_swap()` in
fl2000_ioctl.c — in the ioctl surface-upload path, far from the
render/bulk code, which is why it is easy to miss when porting.

Symptoms without the swap: solid colors look CORRECT (invariant under the
swap — test patterns pass!), but fine detail is scrambled: at 16 bpp text
looks "shredded and re-glued" (2-pixel groups transposed); at 24 bpp the
swap crosses pixel boundaries and produces vertical striping plus
complement-looking hues even in solid areas.

## CRITICAL: 0x8004 format latch programming order

Register 0x8004 must be programmed with SINGLE-BIT read-modify-writes in
the reference driver's exact order — clears first, then pulse the CCS
reset (bit 0), THEN set the color-format bit, then the DAC enable (bit 7).
Combining these into one register write corrupts the format latch (the
readback shows phantom bits and the chip stays/lands in the wrong pixel
width; 16 bpp at high modes can hard-stall the bulk pipe: NAK-forever,
-ETIMEDOUT). This cost a full debugging session — do not "optimize" the
bit-banged sequence.

## Other hard-won operational facts

- Ignore the fake "driver CD" mass-storage interface (usb-storage quirk
  `1d5c:2000:i`): udisks CD polling times out during streaming and the
  SCSI error handler resets the whole device every few seconds.
- 1080p60 at RGB888 (373 MB/s) overruns a 5 Gbps link — the device gets
  hard-reset by the USB core. Use RGB565 above ~320 MB/s.
- Use usb_alloc_coherent + URB_NO_TRANSFER_DMA_MAP for the streaming
  buffers; per-submit DMA mapping at thousands of ops/s corrupted the
  shared xHCI IOMMU domain (DMAR "PTE already set" via btusb).
- Panels behind HDMI→VGA converters typically want 1360x768, not 1366
  (their EDID says so); the 1366 table entry works with h_sync_1
  rewritten for 1360 active pixels.
- EDID reads take ~0.5 s even with tight I2C polling, and the ITE DDC
  bus-hang recovery (`ite_abort_ddc`) can poll for up to 20 s per
  attempt. NOTHING userspace-reachable may touch the DDC/I2C path:
  a cache-miss EDID read inside the GETCONNECTOR ioctl blocked Xorg
  (single-threaded) for ~3.5 minutes on monitor hotplug — the whole
  desktop appears frozen until the cable is pulled. Serve detect() and
  get_modes() from cached state only; do HPD/EDID in a worker and fire
  drm_kms_helper_hotplug_event() on change.
- On X11, Xorg auto-adopts the dongle as a hot-plugged secondary GPU and
  its removal path is crash-prone: unplugging the dongle can segfault
  the X server (NULL deref right after "xf86: remove device"), taking
  the session down. Userspace bug, not driver — Wayland sessions handle
  unplug fine. Explains "X died" reports on plug/unplug (issue #1).

## Confirmed end-to-end (2026-07-04)

640x480@60 RGB565 color bars streamed from Python/pyusb over a USB 2.0
link: 60.0 fps sustained, IT66121 reports VIDEO_STABLE during streaming.

## Reverse-engineered from reference sources (2026-07-11)

Sources: FrescoLogic/FL2000 (official GPL) + klogg/fl2000_drm register map.

- **0x8000 VGA status**: bit1 vga_error(RSC) bit2 lbuf_halt bit3 iso_ack(RSC)
  bit4 td_drop(RSC) bit5 irq_pending(RSC) bit6 pll_status bit7 dac_status
  bit8 lbuf_overflow bit9 lbuf_underflow **bits[25:10] frame_cnt (16-bit
  hardware frame counter)** bit26 hdmi_event(RSC) bit30 monitor_event(RSC)
  bit31 edid_event(RSC). Sticky lbuf bits clear by writing them back.
- **Interrupt EP**: EP3 IN, interface 2, 1 byte, bInterval 6 (4 ms). Fires
  on VGA status events (hotplug, frame drop, lbuf errors); handler reads
  0x8000. Official driver reacts to monitor/EDID events + logs frame drops.
- **0x8004 is the pxclk control reg** (not just format): bit0
  clear_watermark (what we pulse as "CCS reset"), bit1 frame_sync
  (UNUSED by all known drivers — candidate for the EOF/vsync race fix),
  bits2/3 h/v sync polarity, bit6 vga565, bit7 dac_output_en, bit8
  vga_timing_en, bit9 use_new_pkt_retry, bit12 clear_lbuf_status, bit22
  compress, bit24 palette, bit27 disable_halt, bit28 force_de_en, bit29
  vga555.
- **PLL 0x802C**: divisor[7:0] prescaler[9:8] function[14:13]
  multiplier[23:16]; pclk = 10 MHz / prescaler × multiplier / divisor;
  VCO = 10/presc×mult must be 62.5–1000 MHz; function = VCO band
  (<125:0 <250:1 <500:2 else 3). Verified against all 11 table entries.
  → arbitrary modes are synthesizable; the official big table itself goes
  to 1920x1440@60 (234 MHz) and 1920x1200@85 (281 MHz).
- **Sync regs**: 0x8008/0x8010 = active<<16|total; 0x800C = hsync_w<<16 |
  (htotal−hsync_start+1); 0x8014 = start_latency<<20 | vsync_w<<16 |
  (vtotal−vsync_start+1), start_latency conventionally = vstart, bit31 =
  buf_error_en. Encoder validated bit-exact against the CEA 1080p60 row.
- **EOF types**: official EOF_PENDING_BIT=0 / EOF_ZERO_LENGTH=1; bulk path
  uses ZLP. Pending-bit tested on hardware over bulk: chip NAKs everything
  — do not use.
- **Frame pacing**: neither the official Linux driver (app-paced ioctl
  renders) nor klogg (free-run) paces to vsync — the ZLP/vsync race is
  unsolved everywhere, not a regression of ours.

## Frame pacing (anti-blink), what works and what doesn't (2026-07-11)

The EOF/vsync race only fires when delivery is NAK-locked (URB ring
saturated). Fix that shipped: a frequency-locked loop — one frame per
scanout period on an ABSOLUTE deadline chain, period measured from the
hardware frame counter (reg 0x8000 bits[25:10]) each telemetry window,
small P-term on pre-submission ring backlog. Measured on hardware:
race signature eliminated from the wire, 1080p blinks ~20/min → ~5/min.
Residual blinks (both modes) are line-buffer underflows — a separate
bandwidth-margin mechanism; a reduced-blanking mode entry (lower pixel
clock = lower line-drain rate) is the untested candidate.

Hard-won constraints, in blood:
- Relative per-frame sleeps accumulate usleep_range overshoot
  (~0.7 ms/frame!) — absolute deadlines only.
- The PLL-formula period is off by a mode-dependent 0.1–0.6% (chip's
  10 MHz reference is not 10.000 MHz); never trust it as a pacing base,
  measure via the hw frame counter (the chip free-runs, so its counter
  is the true rate even while you under/over-deliver).
- No integrators on ring occupancy: the starve→resync→catch-up path
  reads high occupancy exactly when running slow and winds the integral
  the wrong way (limit-cycle "heartbeat", ~1 s on / 1 s black).
- Sample occupancy BEFORE frame submission, never after.
- The URB ring must exceed the largest frame's URB count (2560x1080@16
  = 86 URBs) or submission is semaphore-throttled to the wire and the
  servo never engages (ring is 128 now).
- frame_sync (0x8004 bit 1): slaves scanout to delivery — refresh
  wobbles with host jitter, ITE drops video-stable. Rejected. The bit
  survives app reset; mode-set must clear it.
- lbuf watermark (0x8030, default 0x0001) has no effect on the race;
  buf_error_en (0x8014 bit31) routes no interrupts. Both dead ends.
- 2560x1080@60.13 (LG DTD timings, PLL 186.0 MHz) added to our mode table
  as first beyond-official-table synthesis — validation pending.

## Isochronous + reduced-blanking investigation (2026-10-23)

Everything below validated on hardware with `probe/fl2000_iso_test.c`, the
userspace libusb harness built this session (bulk + iso bring-up, register
pokes, packet-shape variants, feedback-EP polling, live 0x8000/0x8080/ITE
status; build: `gcc -O2 -o fl2000_iso_test fl2000_iso_test.c $(pkg-config
--cflags --libs libusb-1.0)`, run with the kernel module unloaded+blacklisted).

- **ISO ingestion is impossible on Linux.** Bus delivery is perfect (0 missed
  at IF1 alt 4, 294 MB/s contract; feedback EP 0x82 alive, 128 samples/s) but
  the pixel engine consumes ZERO: feedback reads 0x00000000, lbuf underflows
  continuously, ite never reaches video-stable. Root cause (structural): the
  xHCI makes ONE TD per iso descriptor (xhci-ring.c: `num_tds =
  urb->number_of_packets`) so the USB3 DP-header P bit is 0 at the end of every
  32.4 kB interval — 128 "EOF" markers per frame — while the chip's
  EOF_PENDING_BIT mode (0x803C bit 29) expects Windows-style one-TD-per-URB
  transfers (P=0 once per frame). usbfs caps iso descriptors at ~32 kB
  (32,400 fits by design) so multi-interval TDs can't be built from userspace.
  Fixing it would need an xHCI patch. Dead end — documented, closed.
- **EOF bit semantics**: bit 29 ("pending") also BREAKS the chip's video-out
  generator (ITE never locks); bit 28 (ZLP-style) keeps video-out valid.
  Bulk must use bit 28 (as the reference does); iso+bit28 gives valid syncs
  but still no ingestion.
- **HARDWARE CONSTRAINT (undocumented, bisected)**: the low 16 bits of 0x8014
  (vsync+vbp+1) must be >= ~42 or the scan engine degrades to ~1/3 rate
  (28 → 19 fps, 42 → 60 fps; minimum lies between). True CVT-RB (vblank 31)
  is therefore IMPOSSIBLE on this chip — it needs a big vertical back porch.
- **Custom timings/PLLs otherwise work**: htotal is freely reducible (2080
  scans at full rate) and synthesized PLLs lock exactly (140.0 MHz = VCO 700,
  mult 70/presc 1/div 5 → 59.96 Hz measured, VID-STABLE). The timing regs
  (0x8008/0x800C/0x8010/0x8014) are LIVE — pokes take effect within a frame,
  no app reset needed. The earlier "VCO 830 can't lock" theory was wrong:
  the slow scans were the 0x8014-low16 constraint, not the PLL.
- **But the monitor closes the door**: the Samsung LF27T35 displays only
  CEA/DMT-standard timings — 2080×1125 and 2160×1125 at ~60 Hz produce black
  (both HDMI with VIC 16 and DVI with no infoframe), 2199×1125 produced the
  "Not optimum mode" OSD. CEA 1080p50 (2640×1125) keeps the same pixel clock
  so it gains no drain margin. Intersection of chip (vblank >= 42) and monitor
  (standard modes only) = plain CEA 1080p60. The reduced-drain mode idea is
  dead on this hardware.
- **Diagnostics**: 0x8080 = pixel-engine activity (live counter while
  consuming; 0x02000000 bit 25 stuck = engine idle). 0x807C re-derives
  per-mode packet sizes (reads 32400|32400 at 1080p). 0x801C re-derives after
  timing changes. 0x8000 bit 3 ("iso_ack") never asserts under Linux xHCI.
- **Driver hardened this session**: the iso worker got the 100 ms
  frozen-engine watchdog (validated firing at the counter wrap on hardware)
  plus UNDER/OVER flags and ITE status in its telemetry.
- **USB audit**: the dongle is alone on its 10 Gbps root-hub segment — the
  underflow jitter is intrinsic xHCI/CPU scheduling, not bus contention.

## Open questions / TODO


- Compression details for USB2 at higher modes (fl2000_compression.c)
- Interrupt EP semantics (fl2000_interrupt.c — monitor hotplug events)
- 8-bit palette mode (bit 25/26) — halves USB2 bandwidth needs
- Mainline it66121 bridge driver (drivers/gpu/drm/bridge/ite-it66121.c) as an
  alternative to the reference driver's built-in ITE code for a future DRM
  driver.
