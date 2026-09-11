//! ESP32-S3 GDMA (General DMA controller) model.
//!
//! Register block at `0x6003_F000` (`DR_REG_GDMA_BASE` per
//! `esp32s3.peripherals.ld`). The block is an array
//! of 5 channel pairs (`gdma_dev_t.channel[5]`); each pair is an `in` (RX)
//! block followed by an `out` (TX) block, each 0x60 bytes, so a channel
//! stride is 0xC0 and the `out` block sits at `ch*0xC0 + 0x60`
//! (`soc/gdma_struct.h`).
//!
//! Only the TX (`out`) path is functionally modeled. When software writes
//! `out.link.start` (bit 21 of the OUT link register at out-offset 0x20 —
//! per `gdma_struct.h` `out_link_t`: `addr`[19:0], `stop`=20, `start`=21,
//! `restart`=22, `park`=23) or `in.link.start` (bit 22 of the IN link register
//! at in-offset 0x20 — `gdma_struct.h` `in_link_t`: `addr`[19:0], `auto_ret`=20,
//! `stop`=21, `start`=22, `restart`=23, `park`=24), GDMA
//! walks the descriptor chain starting at `link.addr` (the 20 LSBs of a
//! DRAM descriptor address — full address = `0x3FC0_0000 | addr`, since all
//! DMA descriptors live in DRAM at 0x3FC8_0000..0x3FD0_0000). Each descriptor
//! (`gdma_descriptor_t`) is `{ dw0, buf_addr(dw1), next(dw2), reserved(dw3) }`
//! with `dw0[11:0]=buf_size`, `dw0[23:12]=length` (bytes to transfer),
//! `dw0[30]=eof`, `dw0[31]=owner`. The transfer copies `length` bytes between
//! `buf_addr` and the connected peripheral.
//!
//! For `peri_sel == 9` (RMT, the only GDMA consumer modeled so far) the
//! destination is the RMT TX item RAM: `RMTMEM_BASE + ch*0x100` (each RMT
//! channel block is 0x100 bytes / 64 items). RMT then transmits from that
//! RAM exactly as it does for CPU-written items, raising its own `tx_end`.
//! After the walk, `out_done` (int bit 0) and `out_eof`/`out_total_eof`
//! (bits 1/3) are asserted so the firmware GDMA ISR (and any registered
//! `gdma` tx-event callback, e.g. the esp-idf RMT driver) can run.

use alloc::vec::Vec;

/// GDMA register-block base (APB): `DR_REG_GDMA_BASE = 0x6003_F000` per
/// `esp32s3.peripherals.ld` (`PROVIDE ( GDMA = 0x6003F000 )`). One
/// controller with 5 channel pairs shared by ALL peripherals — RMT, SPI,
/// LCD_CAM, ADC, SHA, AES and I2S alike (the esp-idf crypto drivers allocate
/// channels on this same controller via `esp_crypto_shared_gdma`; there is
/// no separate crypto DMA block, and 0x60042000 is unmapped on S3).
pub const GDMA_BASE: u32 = 0x6003_F000;

/// GDMA per-channel interrupt sources (esp32s3 interrupts.h):
/// RX channels are `ETS_DMA_IN_CH0..4` (66..70), TX channels
/// `ETS_DMA_OUT_CH0..4` (71..75). (There is no combined GDMA source; the
/// old single-source-63 wiring aliased DCACHE_SYNC0 and is removed.)
pub const GDMA_IN_INTR_BASE: u32 = 66;
pub const GDMA_OUT_INTR_BASE: u32 = 71;

/// Number of GDMA channel pairs (TX+RX).
pub const NCH: usize = 5;
/// Byte stride between channel pairs (`in` 0x60 + `out` 0x60).
const CH_STRIDE: u32 = 0xC0;
/// Offset of the `out` block within a channel pair.
const OUT_OFF: u32 = 0x60;

/// Descriptor address region: DRAM descriptors occupy 0x3FC8_0000..0x3FD0_0000,
/// so the upper 12 bits are always 0x3FC and the link register's 20-bit field
/// holds the lower 20 bits (`soc/gdma_struct.h`: "20 least significant bits of
/// the ... descriptor's address").
pub const DESC_BASE: u32 = 0x3FC0_0000;

/// RMT peripheral id for `peri_sel` (`soc/gdma_struct.h` out.peri_sel comment).
pub const GDMA_RMT_PERIPH: u32 = 9;

/// SPI2 / SPI3 peripheral ids for `peri_sel` (`soc/gdma_channel.h`
/// `SOC_GDMA_TRIG_PERIPH_SPI2` = 0, `SOC_GDMA_TRIG_PERIPH_SPI3` = 1). The
/// GDMA `out` channel stages descriptor bytes for a DMA-backed SPI master
/// transfer; the `in` channel copies the captured RX bytes to DRAM once the
/// transfer's trans_done has latched (start the IN link after completion).
pub const GDMA_SPI2_PERIPH: u32 = 0;
pub const GDMA_SPI3_PERIPH: u32 = 1;

/// LCD_CAM peripheral id for `peri_sel` (`soc/gdma_channel.h`
/// `SOC_GDMA_TRIG_PERIPH_LCD0` = 5, shared with CAM0). The GDMA `out`
/// channel streams descriptor words through the LCD TX FIFO as one
/// synchronous 8080 transfer (see `LcdCam::dma_transfer`).
pub const GDMA_LCD_PERIPH: u32 = 5;

/// ADC peripheral id for `peri_sel` (`soc/gdma_channel.h`
/// `SOC_GDMA_TRIG_PERIPH_ADC0` = 8). The GDMA `in` channel drains staged
/// digital-conversion results (`Adc::dma_pop`) to DRAM.
pub const GDMA_ADC_PERIPH: u32 = 8;

/// SHA peripheral id for `peri_sel` (`soc/gdma_channel.h`
/// `SOC_GDMA_TRIG_PERIPH_SHA0`).
pub const GDMA_SHA_PERIPH: u32 = 7;

/// AES peripheral id for `peri_sel` (`soc/gdma_channel.h`
/// `SOC_GDMA_TRIG_PERIPH_AES0`).
pub const GDMA_AES_PERIPH: u32 = 6;

/// I2S0 / I2S1 peripheral ids for `peri_sel` (`soc/gdma_channel.h`
/// `SOC_GDMA_TRIG_PERIPH_I2S0` = 3, `SOC_GDMA_TRIG_PERIPH_I2S1` = 4). The GDMA
/// `out` channel copies descriptor words into the I2S TX FIFO register; the
/// `in` channel copies words out of the I2S RX FIFO register.
pub const GDMA_I2S0_PERIPH: u32 = 3;
pub const GDMA_I2S1_PERIPH: u32 = 4;

pub struct Gdma {
    /// When true, `int_pending` reports any RAW interrupt (ignoring the
    /// per-channel enable). The crypto/shared GDMA (`0x6003F000`) is used by
    /// the esp-idf AES driver in a "polling" mode where it does not enable the
    /// GDMA RX-done interrupt in the matrix, yet still relies on the GDMA ISR
    /// to clear its completion flag — so we must deliver the RAW interrupt
    /// regardless of the enable bit.
    pub ignore_ena: bool,
    // TX (`out`) channel registers.
    out_conf0: [u32; NCH],
    out_conf1: [u32; NCH],
    out_int_raw: [u32; NCH],
    /// TEMP I2S-driver probe (remove): completed-walk counters.
    pub dbg_walks_out: u64,
    pub dbg_walks_in: u64,
    out_int_ena: [u32; NCH],
    out_link: [u32; NCH],
    out_state: [u32; NCH],
    out_peri_sel: [u32; NCH],
    out_eof_des_addr: [u32; NCH],
    /// IN success-EOF descriptor address (IN+0x28, read by the GDMA ISR
    /// to build the RX event).
    in_eof_des_addr: [u32; NCH],
    // RX (`in`) channel registers (modeled for register completeness only).
    in_conf0: [u32; NCH],
    in_conf1: [u32; NCH],
    in_int_raw: [u32; NCH],
    in_int_ena: [u32; NCH],
    in_link: [u32; NCH],
    in_peri_sel: [u32; NCH],

    // Temporary validation ring buffer: records recent raw GDMA writes.
    dbg: [(u32, u32); 64],
    dbg_i: usize,
}

impl Default for Gdma {
    fn default() -> Self {
        Gdma {
            ignore_ena: false,
            out_conf0: [0; NCH],
            out_conf1: [0; NCH],
            out_int_raw: [0; NCH],
            dbg_walks_out: 0,
            dbg_walks_in: 0,
            out_int_ena: [0; NCH],
            out_link: [0; NCH],
            out_state: [0; NCH],
            out_peri_sel: [0; NCH],
            out_eof_des_addr: [0; NCH],
            in_eof_des_addr: [0; NCH],
            in_conf0: [0; NCH],
            in_conf1: [0; NCH],
            in_int_raw: [0; NCH],
            in_int_ena: [0; NCH],
            in_link: [0; NCH],
            in_peri_sel: [0; NCH],
            dbg: [(0u32, 0u32); 64],
            dbg_i: 0,
        }
    }
}

impl Gdma {
    /// OUT channel descriptor link address (full 32-bit DRAM address).
    /// The link register holds the 20 LSBs of the descriptor's DRAM address
    /// (DRAM is 0x3FC80000..0x3FD00000, so the high 12 bits are always 0x3FC).
    /// Its low 2 bits are the start/stop control bits and are not part of the
    /// address; a descriptor is 4-byte aligned so its low 2 bits are 0 anyway.
    pub fn out_link_addr(&self, ch: usize) -> u32 {
        DESC_BASE | ((self.out_link[ch] & 0x000F_FFFF) & !0x3)
    }

    /// IN channel descriptor link address (full 32-bit DRAM address), built
    /// the same way as the OUT link (20 LSBs of the descriptor's DRAM address).
    pub fn in_link_addr(&self, ch: usize) -> u32 {
        DESC_BASE | ((self.in_link[ch] & 0x000F_FFFF) & !0x3)
    }

    /// OUT channel connected peripheral id (`peri_sel.sel`, 6 bits).
    pub fn out_peri_sel(&self, ch: usize) -> u32 {
        self.out_peri_sel[ch] & 0x3F
    }

    /// IN channel connected peripheral id (`peri_sel.sel`, 6 bits).
    pub fn in_peri_sel(&self, ch: usize) -> u32 {
        self.in_peri_sel[ch] & 0x3F
    }

    /// Raw link-start state (the driver may program link before peri_sel;
    /// the SoC retro-arms the I2S pump when peri_sel lands — see soc.rs).
    pub fn out_link_started(&self, ch: usize) -> bool {
        self.out_link[ch] & (1 << 21) != 0
    }
    pub fn in_link_started(&self, ch: usize) -> bool {
        self.in_link[ch] & (1 << 22) != 0
    }

    /// Memory-to-memory mode (TRM `gdma_struct.h` IN_CONF0 `mem_trans_en`,
    /// bit 4): the OUT-link descriptors source DRAM bytes the IN-link
    /// descriptors sink on the same channel pair.
    pub fn in_mem_trans_en(&self, ch: usize) -> bool {
        self.in_conf0[ch] & (1 << 4) != 0
    }

    pub fn set_out_eof_des_addr(&mut self, ch: usize, addr: u32) {
        self.out_eof_des_addr[ch] = addr;
    }

    /// Record the IN success-EOF descriptor address.
    pub fn set_in_eof_des_addr(&mut self, ch: usize, addr: u32) {
        self.in_eof_des_addr[ch] = addr;
    }

    /// Assert `out_done` + `out_eof` + `out_total_eof` for an OUT channel.
    pub fn raise_out_done(&mut self, ch: usize) {
        self.out_int_raw[ch] |= (1 << 0) | (1 << 1) | (1 << 3);
        self.dbg_walks_out += 1; // TEMP I2S-driver probe (remove)
    }

    /// Assert `in_done` + `in_eof` + `in_total_eof` for an IN channel.
    pub fn raise_in_done(&mut self, ch: usize) {
        self.in_int_raw[ch] |= (1 << 0) | (1 << 1) | (1 << 3);
        self.dbg_walks_in += 1; // TEMP I2S-driver probe (remove)
    }

    /// Enable all interrupt bits for an IN channel (so the firmware GDMA ISR
    /// sees a non-zero `INT_ST = RAW & ENA` even when the driver left the
    /// enable at 0, as the AES polling path does).
    pub fn enable_in_int_all(&mut self, ch: usize) {
        self.in_int_ena[ch] = 0xFFFF_FFFF;
    }

    /// Enable all interrupt bits for an OUT channel.
    pub fn enable_out_int_all(&mut self, ch: usize) {
        self.out_int_ena[ch] = 0xFFFF_FFFF;
    }

    /// True if any channel has a pending (raw & enabled) interrupt.
    pub fn int_pending(&self) -> bool {
        for ch in 0..NCH {
            if self.out_int_raw[ch] & self.out_int_ena[ch] != 0
                || (self.ignore_ena && self.out_int_raw[ch] != 0)
            {
                return true;
            }
            if self.in_int_raw[ch] & self.in_int_ena[ch] != 0
                || (self.ignore_ena && self.in_int_raw[ch] != 0)
            {
                return true;
            }
        }
        false
    }

    /// Debug: per-channel (out_peri, out_raw, out_ena, in_peri, in_raw, in_ena).
    pub fn debug_state(&self) -> Vec<(u32, u32, u32, u32, u32, u32)> {
        let mut v = Vec::new();
        for ch in 0..NCH {
            v.push((
                self.out_peri_sel[ch],
                self.out_int_raw[ch],
                self.out_int_ena[ch],
                self.in_peri_sel[ch],
                self.in_int_raw[ch],
                self.in_int_ena[ch],
            ));
        }
        v
    }

    /// Debug: recent raw GDMA writes (offset, value) as a Vec.
    pub fn debug_log(&self) -> Vec<(u32, u32)> {
        let mut v = Vec::with_capacity(64);
        for k in 0..64 {
            let i = (self.dbg_i + k) % 64;
            if self.dbg[i].1 != 0 || self.dbg[i].0 != 0 {
                v.push(self.dbg[i]);
            }
        }
        v
    }

    /// Masked OUT interrupt status for a channel (matrix source 71+ch).
    pub fn out_int_st(&self, ch: usize) -> u32 {
        self.out_int_raw[ch] & self.out_int_ena[ch]
    }

    /// Masked IN interrupt status for a channel (matrix source 66+ch).
    pub fn in_int_st(&self, ch: usize) -> u32 {
        self.in_int_raw[ch] & self.in_int_ena[ch]
    }

    /// Decode an absolute GDMA-page offset into `(is_out, channel, block_off)`.
    fn decode(off: u32) -> Option<(bool, usize, u32)> {
        if off >= NCH as u32 * CH_STRIDE {
            return None;
        }
        let ch = (off / CH_STRIDE) as usize;
        let rem = off % CH_STRIDE;
        if rem < OUT_OFF {
            Some((false, ch, rem))
        } else {
            Some((true, ch, rem - OUT_OFF))
        }
    }

    /// Read a GDMA register. Returns 0 for unmodeled registers / out of range.
    pub fn read32(&self, offset: u32) -> u32 {
        let off = offset & 0xFFF;
        let (is_out, ch, w) = match Self::decode(off) {
            Some(d) => d,
            None => return 0,
        };
        if is_out {
            match w {
                0x00 => self.out_conf0[ch],
                0x04 => self.out_conf1[ch],
                0x08 => self.out_int_raw[ch],
                0x0C => self.out_int_st(ch),
                0x10 => self.out_int_ena[ch],
                0x14 => self.out_int_st(ch), // int_clr is write-only; reads as status
                0x20 => self.out_link[ch],
                0x24 => self.out_state[ch],
                0x28 => self.out_eof_des_addr[ch],
                // peri_sel is +0x48 in both blocks (verified via offsetof).
                0x48 => self.out_peri_sel[ch],
                _ => 0,
            }
        } else {
            match w {
                0x00 => self.in_conf0[ch],
                0x04 => self.in_conf1[ch],
                0x08 => self.in_int_raw[ch],
                0x0C => self.in_int_st(ch),
                0x10 => self.in_int_ena[ch],
                0x14 => self.in_int_st(ch),
                0x20 => self.in_link[ch],
                // IN success-EOF descriptor address (+0x28, read by the
                // GDMA ISR to build RX events).
                0x28 => self.in_eof_des_addr[ch],
                // peri_sel is +0x48 in both blocks (verified via offsetof).
                0x48 => self.in_peri_sel[ch],
                _ => 0,
            }
        }
    }

    /// Write a GDMA register. Returns `Some((ch, is_out))` if the write started
    /// a transfer on channel `ch` (`is_out` = true for an OUT/TX link start,
    /// false for an IN/RX link start). The caller performs the descriptor-walk
    /// copy. Returns `None` for non-start writes.
    pub fn write32(&mut self, offset: u32, value: u32) -> Option<(usize, bool)> {
        let off = offset & 0xFFF;
        self.dbg[self.dbg_i] = (off, value);
        self.dbg_i = (self.dbg_i + 1) % 64;
        let (is_out, ch, w) = Self::decode(off)?;
        if is_out {
            match w {
                0x00 => self.out_conf0[ch] = value,
                0x04 => self.out_conf1[ch] = value,
                // int_raw is hardware-set / R/WTC; ignore software writes.
                0x08 => {}
                0x10 => self.out_int_ena[ch] = value,
                0x14 => self.out_int_raw[ch] &= !value, // int_clr clears raw bits
                0x20 => {
                    self.out_link[ch] = value;
                    if value & (1 << 21) != 0 {
                        return Some((ch, true));
                    }
                }
                // peri_sel is +0x48 in both blocks (verified via offsetof).
                0x48 => self.out_peri_sel[ch] = value,
                _ => {}
            }
        } else {
            match w {
                0x00 => self.in_conf0[ch] = value,
                0x04 => self.in_conf1[ch] = value,
                0x08 => {}
                0x10 => self.in_int_ena[ch] = value,
                0x14 => self.in_int_raw[ch] &= !value,
                0x20 => {
                    self.in_link[ch] = value;
                    // RX start begins a peripheral->memory transfer; the caller
                    // performs the descriptor-walk copy from the peripheral.
                    // IN_LINK `start` is bit 22 (gdma_struct.h in_link_t:
                    // addr[19:0], auto_ret=20, stop=21, start=22, restart=23,
                    // park=24); OUT_LINK `start` is bit 21 (out_link_t).
                    if value & (1 << 22) != 0 {
                        self.out_state[ch] &= !(1 << 1);
                        return Some((ch, false));
                    }
                }
                // IN peri_sel is +0x48 like OUT (verified via offsetof).
                0x48 => self.in_peri_sel[ch] = value,
                _ => {}
            }
        }
        None
    }
}
