//! ESP32-S3 LCD_CAM controller (= parallel I/O / "PARLIO").
//!
//! Base `DR_REG_LCD_CAM_BASE = 0x6004_1000`. Register layout per esp-idf
//! `components/soc/esp32s3/register/soc/lcd_cam_reg.h`.
//!
//! Functional model: a TX/RX FIFO pair plus a transfer-start / transfer-done
//! interrupt path. Writing words to `LCD_DATA` (0x40) pushes the TX FIFO;
//! `LCD_FIFO_STATUS` (0x44) reports the count. Setting `LCD_START` (bit 27 of
//! `LCD_USER` 0x14) begins a parallel transfer: each TX FIFO word is presented
//! on the LCD_CAM GPIO-matrix data signals (`LCD_DATA_OUT0..15` = 133..148) for
//! one `LCD_PCLK` (sig 154) cycle, toggling PCLK and asserting `LCD_CS` (sig
//! 132, active low) and `LCD_DC` (sig 153, from `LCD_USER` bit 26) for the
//! duration. When the FIFO empties `LCD_TRANS_DONE` (bit 1 of the
//! `LC_DMA_INT_*` block at 0x64/0x68/0x6C/0x70) is raised.
//!
//! The camera (RX) path captures a host-staged frame: `cam_inject_frame`
//! stages one frame (list of words, e.g. from a virtual camera device); a
//! `CAM_START` write (bit 29 of `CAM_CTRL1` 0x08, verified in lcd_cam_reg.h)
//! arms/starts capture. With no staged data the controller waits (models the
//! VSYNC wait for a real sensor); once data is present it asserts VSYNC
//! (`CAM_VSYNC_INT`, bit 2) and shifts one word per tick into the RX FIFO
//! (read via `CAM_DATA` 0x48 / `CAM_FIFO_STATUS` 0x4C). Every
//! `LINE_INT_NUM+1` (`CAM_CTRL1` [21:16]) words raise `CAM_HS_INT` (bit 3,
//! "received-lines" semantics); a nonzero `REC_DATA_BYTELEN` (`CAM_CTRL1`
//! [15:0]) ends capture after that many bytes + 1 (GDMA-eof semantics).
//! When the frame is consumed the capture ends and `CAM_START` self-clears.
//! A full RX FIFO stalls the stream (flow control); firmware draining
//! resumes it. `CAM_CTRL` bit 5 (`CAM_BYTE_ORDER`) byte-swaps each word.
//! `CAM_RESET` (CTRL1.30) / `CAM_AFIFO_RESET` (CTRL1.31) clear RX state.
//!
//! GDMA-RX transport streams captured words into IN descriptors (peri 5)
//! via a cursor pump (see `Soc::poll_cam_dma`); `CAM_STOP_EN` (CAM_CTRL
//! bit 0: stop when the GDMA staging FIFO is full) is honored. Use one
//! path per capture (polling fills RX FIFO, DMA fills descriptors).
//! NOT modeled: clock-divider timing
//! (one word per tick), 2BYTE packing (injected words already are units).
//!
//! The presented signals are observable on GPIO pins whose `FUNC_OUT_SEL` is
//! routed to the corresponding LCD_CAM signal index (see `signal_level`);
//! during RX capture (TX idle) the CAM input levels (VSYNC/PCLK/DATA) show
//! instead so a sensor-driven pad reads back like silicon.

pub const LCD_CAM_BASE: u32 = 0x6004_1000;

/// LCD_CAM interrupt source for the matrix (ETS_LCD_CAM_INTR_SOURCE = 24).
pub const LCD_CAM_INTR_SOURCE: u32 = 24;

// LCD_CAM GPIO-matrix signal indices (gpio_sig_map.h).
const SIG_LCD_CS: u32 = 132;
const SIG_DATA0: u32 = 133; // .. SIG_DATA15 = 148
const SIG_H_ENABLE: u32 = 150;
const SIG_H_SYNC: u32 = 151;
const SIG_V_SYNC: u32 = 152;
const SIG_LCD_DC: u32 = 153;
const SIG_LCD_PCLK: u32 = 154;
const SIG_CAM_PCLK: u32 = 149; // CAM_PCLK/CAM_CLK shared PCLK

const REG_COUNT: usize = 0x1000 / 4;

const LCD_USER: u32 = 0x14;
const LCD_CMD_BIT: u32 = 1 << 26;
const CAM_CTRL1: u32 = 0x08;
pub const LCD_DATA: u32 = 0x40;
const LCD_FIFO_STATUS: u32 = 0x44;
const CAM_DATA: u32 = 0x48;
const CAM_FIFO_STATUS: u32 = 0x4C;
const LC_DMA_INT_ENA: u32 = 0x64;
const LC_DMA_INT_RAW: u32 = 0x68;
const LC_DMA_INT_ST: u32 = 0x6C;
const LC_DMA_INT_CLR: u32 = 0x70;

const LCD_START_BIT: u32 = 1 << 27;
const LCD_RESET_BIT: u32 = 1 << 28;
const CAM_START_BIT: u32 = 1 << 29; // CAM_CTRL1: camera module start
const CAM_RESET_BIT: u32 = 1 << 30; // CAM_CTRL1: camera module reset
const CAM_AFIFO_RESET_BIT: u32 = 1 << 31; // CAM_CTRL1: async RX FIFO reset
const CAM_CTRL: u32 = 0x04;
const CAM_BYTE_ORDER_BIT: u32 = 1 << 5; // CAM_CTRL: swap bytes per word
// CAM_CTRL bit 0: stop capture when the GDMA staging FIFO is full
// (lcd_cam_reg.h CAM_STOP_EN); without it overruns drop words.
const CAM_STOP_EN_BIT: u32 = 1 << 0;

const LCD_TRANS_DONE: u32 = 1 << 1;
const CAM_VSYNC_INT: u32 = 1 << 2;
const CAM_HS_INT: u32 = 1 << 3;
const LCD_VSYNC_INT: u32 = 1 << 0;
const INT_MASK: u32 = LCD_VSYNC_INT | LCD_TRANS_DONE | CAM_VSYNC_INT | CAM_HS_INT;

const FIFO_DEPTH: usize = 16;

pub struct LcdCam {
    regs: [u32; REG_COUNT],
    tx_fifo: [u32; FIFO_DEPTH],
    tx_count: usize,
    rx_fifo: [u32; FIFO_DEPTH],
    rx_count: usize,
    /// GDMA-RX staging FIFO (capture words bound for IN descriptors).
    dma_fifo: alloc::collections::VecDeque<u32>,
    /// GDMA-RX path armed (IN cursor active): tick_cam streams here
    /// instead of `rx_fifo`.
    dma_active: bool,
    int_raw: u32,
    int_ena: u32,
    // Parallel-transfer output state.
    busy: bool,
    cur_word: u32,
    pclk: u32,
    dc: u32,
    // Camera-capture input state.
    capturing: bool, // CAM_START latched (armed or streaming)
    streaming: bool, // frame data present (VSYNC asserted)
    staged: alloc::collections::VecDeque<alloc::vec::Vec<u32>>, // host frames
    cur_frame: alloc::vec::Vec<u32>, // frame being streamed
    cur_pos: usize,  // next word index in cur_frame
    line_pos: usize, // words since last HSYNC
    byte_count: u32, // bytes streamed (for REC_DATA_BYTELEN)
    cam_pclk: u32,
    cam_vsync: u32,
    cam_hsync: u32,
    cam_word: u32, // current input word (DATA_IN level source)
}

impl LcdCam {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            regs: [0; REG_COUNT],
            tx_fifo: [0; FIFO_DEPTH],
            tx_count: 0,
            rx_fifo: [0; FIFO_DEPTH],
            rx_count: 0,
            dma_fifo: alloc::collections::VecDeque::new(),
            dma_active: false,
            int_raw: 0,
            int_ena: 0,
            busy: false,
            cur_word: 0,
            pclk: 0,
            dc: 0,
            capturing: false,
            streaming: false,
            staged: alloc::collections::VecDeque::new(),
            cur_frame: alloc::vec::Vec::new(),
            cur_pos: 0,
            line_pos: 0,
            byte_count: 0,
            cam_pclk: 0,
            cam_vsync: 0,
            cam_hsync: 0,
            cam_word: 0,
        }
    }

    /// Stage one camera frame (words) from the host (virtual camera device).
    /// Frames queue; each `CAM_START` capture consumes the next one.
    pub fn cam_inject_frame(&mut self, words: &[u32]) {
        self.staged.push_back(words.into());
    }

    /// True while a camera capture is armed or streaming (for tick gating).
    pub fn cam_active(&self) -> bool {
        self.capturing
    }

    fn idx(off: u32) -> usize {
        ((off & 0xFFF) / 4) as usize
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let o = offset & 0xFFF;
        match o {
            LCD_DATA => 0, // write-only push register
            LCD_FIFO_STATUS => {
                let c = (self.tx_count as u32) & 0x7FF;
                c | (c << 16)
            }
            CAM_DATA => {
                if self.rx_count > 0 {
                    let v = self.rx_fifo[0];
                    for i in 1..self.rx_count {
                        self.rx_fifo[i - 1] = self.rx_fifo[i];
                    }
                    self.rx_count -= 1;
                    v
                } else {
                    0
                }
            }
            CAM_FIFO_STATUS => {
                let c = (self.rx_count as u32) & 0x7FF;
                c | (c << 16)
            }
            LC_DMA_INT_RAW => self.int_raw,
            LC_DMA_INT_ST => self.int_raw & self.int_ena,
            LC_DMA_INT_ENA => self.int_ena,
            _ => self.regs[Self::idx(o)],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let o = offset & 0xFFF;
        match o {
            LCD_DATA => {
                if self.tx_count < FIFO_DEPTH {
                    self.tx_fifo[self.tx_count] = value;
                    self.tx_count += 1;
                }
            }
            LCD_FIFO_STATUS | CAM_FIFO_STATUS | CAM_DATA => { /* read-only */ }
            LC_DMA_INT_ENA => self.int_ena = value & INT_MASK,
            LC_DMA_INT_CLR => self.int_raw &= !value,
            LC_DMA_INT_RAW => { /* read-only */ }
            LCD_USER => {
                self.regs[Self::idx(o)] = value;
                if value & LCD_RESET_BIT != 0 {
                    self.tx_count = 0;
                    self.int_raw = 0;
                    self.busy = false;
                    self.pclk = 0;
                }
                if value & LCD_START_BIT != 0 {
                    // Begin a parallel transfer.
                    self.busy = true;
                    self.pclk = 0;
                    self.dc = (value & LCD_CMD_BIT) >> 26;
                    self.cur_word = 0;
                    // Present the first word on the next PCLK rising edge.
                }
            }
            CAM_CTRL1 => {
                self.regs[Self::idx(o)] = value;
                if value & CAM_RESET_BIT != 0 {
                    self.rx_count = 0;
                    self.end_capture();
                }
                if value & CAM_AFIFO_RESET_BIT != 0 {
                    // Async FIFO reset: drop received (not yet read) words.
                    self.rx_count = 0;
                }
                if value & CAM_START_BIT != 0 {
                    // Arm (or restart) a capture; streams once framedata is
                    // staged (VSYNC wait for a real sensor).
                    self.capturing = true;
                    self.streaming = false;
                }
            }
            _ => {
                self.regs[Self::idx(o)] = value;
            }
        }
    }

    /// Advance the parallel transfer by one emulator step (one PCLK half-cycle).
    /// True mid-transfer. The SoC skips `tick()` otherwise — `tick`
    /// returns immediately when idle, so gating is behavior-preserving.
    pub fn is_active(&self) -> bool {
        self.busy || self.capturing
    }

    /// Latch a capture shut: VSYNC off, START self-cleared (lets firmware
    /// poll START as a busy flag, like SPI's self-clearing usr bit).
    fn end_capture(&mut self) {
        self.capturing = false;
        self.streaming = false;
        self.cam_vsync = 0;
        self.cam_hsync = 0;
        self.regs[Self::idx(CAM_CTRL1)] &= !CAM_START_BIT;
    }

    fn line_int_num(&self) -> usize {
        ((self.regs[Self::idx(CAM_CTRL1)] >> 16) & 0x3F) as usize
    }

    fn rec_bytelen(&self) -> u32 {
        self.regs[Self::idx(CAM_CTRL1)] & 0xFFFF
    }

    fn byte_swap(&self) -> bool {
        self.regs[Self::idx(CAM_CTRL)] & CAM_BYTE_ORDER_BIT != 0
    }

    /// Arm/disarm the GDMA-RX path (IN cursor active): captured words
    /// stream into the staging FIFO instead of `rx_fifo`.
    pub fn set_dma_active(&mut self, on: bool) {
        self.dma_active = on;
        if !on {
            self.dma_fifo.clear();
        }
    }

    /// Pop a staged GDMA word for the descriptor pump (None when dry).
    pub fn dma_pop(&mut self) -> Option<u32> {
        self.dma_fifo.pop_front()
    }

    /// Staged GDMA words available.
    pub fn dma_pending(&self) -> usize {
        self.dma_fifo.len()
    }

    /// Capture running (set by CAM_START, cleared at end-of-frame).
    pub fn is_capturing(&self) -> bool {
        self.capturing
    }

    pub fn tick(&mut self) {
        if self.busy {
            self.pclk ^= 1;
            if self.pclk == 1 {
                // Rising PCLK edge: present the next TX FIFO word.
                if self.tx_count > 0 {
                    self.cur_word = self.tx_fifo[0];
                    for i in 1..self.tx_count {
                        self.tx_fifo[i - 1] = self.tx_fifo[i];
                    }
                    self.tx_count -= 1;
                } else {
                    // FIFO drained: finish the transfer.
                    self.busy = false;
                    self.int_raw |= LCD_TRANS_DONE;
                }
            }
        }
        if self.capturing {
            self.tick_cam();
        }
    }

    /// One camera-capture step: open the frame (VSYNC), shift a word per
    /// tick unless the RX FIFO is full (flow control), HSYNC per line,
    /// REC_DATA_BYTELEN truncation, end-of-frame shutdown.
    fn tick_cam(&mut self) {
        if !self.streaming {
            // VSYNC wait: begin once a staged frame exists.
            if let Some(frame) = self.staged.pop_front() {
                self.cur_frame = frame;
                self.cur_pos = 0;
                self.line_pos = 0;
                self.byte_count = 0;
                self.streaming = true;
                self.cam_vsync = 1;
                self.int_raw |= CAM_VSYNC_INT;
            } else {
                return;
            }
        }
        self.cam_pclk ^= 1;
        if !self.dma_active && self.rx_count >= FIFO_DEPTH {
            return; // backpressure: firmware must drain CAM_DATA first
        }
        if self.dma_active && self.dma_fifo.len() >= FIFO_DEPTH {
            // GDMA staging full: end the capture with STOP_EN, else stall
            // (retry next tick — no loss) until the pump drains.
            if self.regs[Self::idx(CAM_CTRL)] & CAM_STOP_EN_BIT != 0 {
                self.end_capture();
            }
            return;
        }
        let limit = self.rec_bytelen();
        if limit != 0 && self.byte_count > limit {
            self.end_capture();
            return;
        }
        if self.cur_pos >= self.cur_frame.len() {
            self.end_capture();
            return;
        }
        let mut w = self.cur_frame[self.cur_pos];
        if self.byte_swap() {
            w = w.swap_bytes();
        }
        self.cur_pos += 1;
        self.byte_count += 4;
        if self.dma_active {
            self.dma_fifo.push_back(w);
        } else {
            self.rx_fifo[self.rx_count] = w;
            self.rx_count += 1;
        }
        self.cam_word = w;
        self.line_pos += 1;
        if self.line_pos > self.line_int_num() {
            self.line_pos = 0;
            self.cam_hsync = 1;
            self.int_raw |= CAM_HS_INT;
        } else {
            self.cam_hsync = 0;
        }
    }

    /// GDMA-fed block transfer (`peri_sel` LCD): stream `words` through the
    /// TX FIFO, ticking PCLK to drain (the FIFO holds 16 words; transfers
    /// may be longer), finishing with LCD_TRANS_DONE latched. One
    /// synchronous DMA-backed 8080 transfer: firmware cannot observe
    /// mid-transfer states within the GDMA walk's single step, so this is
    /// observationally identical to an async completion (unlike SPI, where
    /// the live waveform is the validation target).
    pub fn dma_transfer(&mut self, words: &[u32]) {
        self.write32(LCD_USER, LCD_START_BIT);
        let mut i = 0;
        while i < words.len() || self.busy {
            while i < words.len() && self.tx_count < FIFO_DEPTH {
                self.write32(LCD_DATA, words[i]);
                i += 1;
            }
            // One full PCLK cycle presents a word.
            self.tick();
            self.tick();
        }
    }

    /// Interrupt status = RAW & ENA (LC_DMA_INT_ST).
    pub fn int_st(&self) -> u32 {
        self.int_raw & self.int_ena
    }

    /// Current level (0/1) of an LCD_CAM GPIO-matrix signal index. While a
    /// TX transfer runs the LCD output view wins; while an RX capture runs
    /// (TX idle) the camera input view shows (a sensor-driven pad reads
    /// back through `cam_input_level`); idle reads CS high, rest low.
    pub fn signal_level(&self, sig: u32) -> u32 {
        if self.busy {
            if sig == SIG_LCD_CS {
                0
            } else if (SIG_DATA0..=SIG_DATA0 + 15).contains(&sig) {
                (self.cur_word >> (sig - SIG_DATA0)) & 1
            } else if sig == SIG_LCD_PCLK || sig == SIG_CAM_PCLK {
                self.pclk
            } else if sig == SIG_LCD_DC {
                self.dc
            } else if sig == SIG_H_ENABLE || sig == SIG_H_SYNC || sig == SIG_V_SYNC {
                1
            } else {
                0
            }
        } else if self.capturing {
            self.cam_input_level(sig)
        } else if sig == SIG_LCD_CS {
            1
        } else {
            0
        }
    }

    /// Camera (sensor-driven) input level for a matrix signal index: VSYNC
    /// high mid-frame, HSYNC pulsed per line, PCLK toggling, DATA = the
    /// current input word. Consulted for pads whose input routing selects a
    /// CAM signal (149..152, 133..148) while a capture runs.
    pub fn cam_input_level(&self, sig: u32) -> u32 {
        if sig == SIG_V_SYNC {
            self.cam_vsync
        } else if sig == SIG_H_SYNC {
            self.cam_hsync
        } else if sig == SIG_H_ENABLE {
            u32::from(self.streaming)
        } else if sig == SIG_CAM_PCLK {
            self.cam_pclk
        } else if (SIG_DATA0..=SIG_DATA0 + 15).contains(&sig) {
            (self.cam_word >> (sig - SIG_DATA0)) & 1
        } else {
            0
        }
    }

    /// True while camera inputs are driven (for input-overlay gating).
    pub fn cam_driving(&self) -> bool {
        self.capturing
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn tx_fifo_counts_and_drains_on_start() {
        let mut d = LcdCam::new();
        d.write32(LCD_DATA, 0x1111_1111);
        d.write32(LCD_DATA, 0x2222_2222);
        d.write32(LCD_DATA, 0x3333_3333);
        assert_eq!(d.read32(LCD_FIFO_STATUS) & 0x7FF, 3, "TX FIFO count");
        d.write32(LC_DMA_INT_ENA, LCD_TRANS_DONE);
        d.write32(LC_DMA_INT_CLR, 0xF);
        d.write32(LCD_USER, LCD_START_BIT);
        // Each TX word is presented over one PCLK cycle (~2 ticks); after the
        // FIFO drains the next rising edge finishes and raises TRANS_DONE.
        for _ in 0..32 {
            d.tick();
        }
        assert_eq!(d.read32(LCD_FIFO_STATUS) & 0x7FF, 0, "TX drained");
        assert_eq!(d.read32(LC_DMA_INT_RAW) & LCD_TRANS_DONE, LCD_TRANS_DONE);
        assert_eq!(d.read32(LC_DMA_INT_ST) & LCD_TRANS_DONE, LCD_TRANS_DONE);
        // Clear and confirm.
        d.write32(LC_DMA_INT_CLR, LCD_TRANS_DONE);
        assert_eq!(d.read32(LC_DMA_INT_RAW) & LCD_TRANS_DONE, 0, "RAW cleared");
    }

    /// GDMA-fed transfer longer than the 16-word FIFO completes with
    /// TRANS_DONE latched and the FIFO drained (PCLK ticks interleave).
    #[test]
    fn dma_transfer_streams_past_fifo_depth() {
        let mut d = LcdCam::new();
        let words: Vec<u32> = (0..40).collect();
        d.dma_transfer(&words);
        assert_eq!(d.read32(LCD_FIFO_STATUS) & 0x7FF, 0, "TX drained");
        assert_eq!(
            d.read32(LC_DMA_INT_RAW) & LCD_TRANS_DONE,
            LCD_TRANS_DONE,
            "TRANS_DONE latched"
        );
    }

    #[test]
    fn parallel_signals_driven_during_transfer() {
        let mut d = LcdCam::new();
        d.write32(LCD_DATA, 0x0000_FFF0); // data lines 4..15 high
        d.write32(LCD_USER, LCD_START_BIT | LCD_CMD_BIT);
        assert_eq!(d.signal_level(SIG_LCD_CS), 0, "CS active (low) at start");
        assert_eq!(d.signal_level(SIG_LCD_DC), 1, "DC = cmd bit");
        d.tick();
        d.tick(); // present first word
        for i in 4..16 {
            assert_eq!(d.signal_level(SIG_DATA0 + i as u32), 1, "data line {}", i);
        }
        assert_eq!(d.signal_level(SIG_DATA0), 0);
    }

    #[test]
    fn config_registers_round_trip() {
        let mut d = LcdCam::new();
        d.write32(0x10, 0xABCD_0000);
        d.write32(0x24, 0x1234_5678);
        assert_eq!(d.read32(0x10), 0xABCD_0000);
        assert_eq!(d.read32(0x24), 0x1234_5678);
    }

    const CAM_START: u32 = 1 << 29;
    const _CAM_RESET: u32 = 1 << 30;
    const CAM_AFIFO_RESET: u32 = 1 << 31;

    /// START with nothing staged arms the capture (VSYNC wait): no VSYNC,
    /// no FIFO data, still capturing.
    #[test]
    fn cam_start_without_frame_waits_for_vsync() {
        let mut d = LcdCam::new();
        d.write32(CAM_CTRL1, CAM_START);
        assert!(d.cam_active(), "armed");
        for _ in 0..8 {
            d.tick();
        }
        assert_eq!(d.read32(CAM_FIFO_STATUS) & 0x7FF, 0, "no data yet");
        assert_eq!(d.read32(LC_DMA_INT_RAW) & CAM_VSYNC_INT, 0, "no VSYNC yet");
        assert!(d.cam_active(), "still armed");
    }

    /// START with a staged frame streams it: VSYNC + FIFO words in order +
    /// HSYNC per LINE_INT_NUM+1 + START self-clear at end.
    #[test]
    fn cam_capture_streams_staged_frame() {
        let mut d = LcdCam::new();
        // One line of 4 words (LINE_INT_NUM = 3).
        d.write32(CAM_CTRL1, (3 << 16) | CAM_START);
        d.cam_inject_frame(&[0x0102_0304, 0x1111_1111, 0x2222_2222, 0x3333_3333]);
        for _ in 0..8 {
            d.tick();
        }
        assert_eq!(d.read32(CAM_FIFO_STATUS) & 0x7FF, 4, "frame arrived");
        assert_eq!(d.read32(LC_DMA_INT_RAW) & CAM_VSYNC_INT, CAM_VSYNC_INT);
        assert_eq!(d.read32(LC_DMA_INT_RAW) & CAM_HS_INT, CAM_HS_INT);
        assert_eq!(d.read32(CAM_CTRL1) & CAM_START, 0, "START self-cleared");
        assert!(!d.cam_active(), "capture ended");
        assert_eq!(d.read32(CAM_DATA), 0x0102_0304);
        assert_eq!(d.read32(CAM_DATA), 0x1111_1111);
        assert_eq!(d.read32(CAM_DATA), 0x2222_2222);
        assert_eq!(d.read32(CAM_DATA), 0x3333_3333);
        assert_eq!(d.read32(CAM_DATA), 0, "FIFO empty reads 0");
    }

    /// A full RX FIFO stalls the stream (flow control); draining resumes it.
    #[test]
    fn cam_backpressure_stalls_until_drained() {
        let mut d = LcdCam::new();
        d.write32(CAM_CTRL1, CAM_START);
        let frame: Vec<u32> = (0..20).collect();
        d.cam_inject_frame(&frame);
        for _ in 0..64 {
            d.tick();
        }
        assert_eq!(d.read32(CAM_FIFO_STATUS) & 0x7FF, 16, "FIFO full, stalled");
        assert!(d.cam_active(), "still capturing (4 words pending)");
        for i in 0..16u32 {
            assert_eq!(d.read32(CAM_DATA), i);
        }
        for _ in 0..16 {
            d.tick();
        }
        for i in 16..20u32 {
            assert_eq!(d.read32(CAM_DATA), i);
        }
        assert!(!d.cam_active(), "drained to end");
    }

    /// REC_DATA_BYTELEN truncates the capture (GDMA-eof semantics: N+1 bytes).
    #[test]
    fn cam_rec_bytelen_truncates_capture() {
        let mut d = LcdCam::new();
        d.write32(CAM_CTRL1, 15 | CAM_START); // 16 bytes = 4 words
        d.cam_inject_frame(&[1, 2, 3, 4, 5, 6, 7, 8]);
        for _ in 0..32 {
            d.tick();
        }
        assert_eq!(d.read32(CAM_FIFO_STATUS) & 0x7FF, 4, "truncated to 4");
        assert!(!d.cam_active(), "capture ended at the limit");
        assert_eq!(d.read32(CAM_DATA), 1);
    }

    /// CAM_CTRL bit 5 byte-swaps each received word.
    #[test]
    fn cam_byte_order_swaps_words() {
        let mut d = LcdCam::new();
        d.write32(CAM_CTRL, 1 << 5);
        d.write32(CAM_CTRL1, CAM_START);
        d.cam_inject_frame(&[0x0102_0304]);
        for _ in 0..4 {
            d.tick();
        }
        assert_eq!(d.read32(CAM_DATA), 0x0403_0201);
    }

    /// AFIFO reset drops received words; START stays armed for a retry.
    #[test]
    fn cam_afifo_reset_drops_fifo() {
        let mut d = LcdCam::new();
        d.write32(CAM_CTRL1, CAM_START);
        d.cam_inject_frame(&[0xAA, 0xBB]);
        for _ in 0..8 {
            d.tick();
        }
        assert_eq!(d.read32(CAM_FIFO_STATUS) & 0x7FF, 2);
        d.write32(CAM_CTRL1, CAM_AFIFO_RESET | CAM_START);
        assert_eq!(d.read32(CAM_FIFO_STATUS) & 0x7FF, 0, "FIFO dropped");
    }

    /// VSYNC input level is observable while streaming (sensor loopback).
    #[test]
    fn cam_vsync_visible_during_capture() {
        let mut d = LcdCam::new();
        assert_eq!(d.cam_input_level(SIG_V_SYNC), 0, "idle low");
        d.write32(CAM_CTRL1, CAM_START);
        d.cam_inject_frame(&[0x0000_0010, 0x0000_0010, 0x0000_0010, 0x0000_0010]);
        d.tick();
        assert_eq!(d.cam_input_level(SIG_V_SYNC), 1, "VSYNC mid-frame");
        assert_eq!(d.cam_input_level(SIG_DATA0 + 4), 1, "data bit 4");
        assert_eq!(d.cam_input_level(SIG_DATA0), 0, "data bit 0");
    }
}

#[cfg(test)]
mod dma_tests {
    use super::*;
    use alloc::vec::Vec;

    fn dma_dev() -> LcdCam {
        let mut d = LcdCam::new();
        d.cam_inject_frame(&[0x1111_1111, 0x2222_2222]);
        d.set_dma_active(true);
        d.write32(CAM_CTRL1, CAM_START_BIT);
        d
    }

    #[test]
    fn dma_path_streams_into_staging_not_rx_fifo() {
        let mut d = dma_dev();
        for _ in 0..16 {
            d.tick();
        }
        assert_eq!(d.dma_pending(), 2, "both words staged for GDMA");
        assert_eq!(d.read32(CAM_FIFO_STATUS) & 0x7FF, 0, "RX FIFO untouched");
        assert_eq!(d.dma_pop(), Some(0x1111_1111));
        assert_eq!(d.dma_pop(), Some(0x2222_2222));
        assert_eq!(d.dma_pop(), None, "staging drained");
    }

    #[test]
    fn stop_en_ends_capture_on_full_staging() {
        let mut d = LcdCam::new();
        // 20-word frame (over the 16-word staging cap), STOP_EN set, no
        // pump drain: staging fills, then the capture self-clears START.
        let frame: Vec<u32> = (0..20).collect();
        d.cam_inject_frame(&frame);
        d.set_dma_active(true);
        d.write32(CAM_CTRL, CAM_STOP_EN_BIT);
        d.write32(CAM_CTRL1, CAM_START_BIT);
        for _ in 0..64 {
            d.tick();
        }
        assert_eq!(d.dma_pending(), 16, "staging capped");
        assert_eq!(
            d.read32(CAM_CTRL1) & CAM_START_BIT,
            0,
            "START self-cleared on full staging with STOP_EN"
        );
        assert!(!d.is_capturing(), "capture ended");
    }

    #[test]
    fn overrun_without_stop_en_keeps_capturing() {
        let mut d = LcdCam::new();
        let frame: Vec<u32> = (0..20).collect();
        d.cam_inject_frame(&frame);
        d.set_dma_active(true);
        d.write32(CAM_CTRL1, CAM_START_BIT); // STOP_EN clear
        for _ in 0..64 {
            d.tick();
        }
        assert_eq!(d.dma_pending(), 16, "staging capped");
        assert_ne!(
            d.read32(CAM_CTRL1) & CAM_START_BIT,
            0,
            "START held without STOP_EN"
        );
    }
}
