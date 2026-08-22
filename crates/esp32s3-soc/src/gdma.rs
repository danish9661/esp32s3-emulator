//! ESP32-S3 GDMA (General DMA controller) model.
//!
//! Register block at `0x6004_2000` (TRM GDMA chapter). The block is an array
//! of 5 channel pairs (`gdma_dev_t.channel[5]`); each pair is an `in` (RX)
//! block followed by an `out` (TX) block, each 0x60 bytes, so a channel
//! stride is 0xC0 and the `out` block sits at `ch*0xC0 + 0x60`
//! (`soc/gdma_struct.h`).
//!
//! Only the TX (`out`) path is functionally modeled. When software writes
//! `out.link.start` (bit 21 of the link register at out-offset 0x20 — the
//! `addr` field is bits [19:0], then `stop`=20, `start`=21, `restart`=22,
//! `park`=23, per `gdma_struct.h`), GDMA
//! walks the descriptor chain starting at `out.link.addr` (the 20 LSBs of a
//! DRAM descriptor address — full address = `0x3FC0_0000 | addr`, since all
//! DMA descriptors live in DRAM at 0x3FC8_0000..0x3FD0_0000). Each descriptor
//! (`gdma_descriptor_t`) is `{ dw0, buf_addr(dw1), next(dw2), reserved(dw3) }`
//! with `dw0[11:0]=buf_size`, `dw0[23:12]=length` (bytes to transfer),
//! `dw0[30]=eof`, `dw0[31]=owner`. The transfer copies `length` bytes from
//! `buf_addr` into the connected peripheral.
//!
//! For `peri_sel == 9` (RMT, the only GDMA consumer modeled so far) the
//! destination is the RMT TX item RAM: `RMTMEM_BASE + ch*0x100` (each RMT
//! channel block is 0x100 bytes / 64 items). RMT then transmits from that
//! RAM exactly as it does for CPU-written items, raising its own `tx_end`.
//! After the walk, `out_done` (int bit 0) and `out_eof`/`out_total_eof`
//! (bits 1/3) are asserted so the firmware GDMA ISR (and any registered
//! `gdma` tx-event callback, e.g. the esp-idf RMT driver) can run.

/// GDMA register-block base (APB).
pub const GDMA_BASE: u32 = 0x6004_2000;

/// GDMA interrupt source for the interrupt matrix (esp32s3 interrupts.h
/// `ETS_GDMA_INTR_SOURCE = 63`).
pub const GDMA_INTR_SOURCE: u32 = 63;

/// Number of GDMA channel pairs (TX+RX).
const NCH: usize = 5;
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

#[derive(Default)]
pub struct Gdma {
    // TX (`out`) channel registers.
    out_conf0: [u32; NCH],
    out_conf1: [u32; NCH],
    out_int_raw: [u32; NCH],
    out_int_ena: [u32; NCH],
    out_link: [u32; NCH],
    out_state: [u32; NCH],
    out_peri_sel: [u32; NCH],
    out_eof_des_addr: [u32; NCH],
    // RX (`in`) channel registers (modeled for register completeness only).
    in_conf0: [u32; NCH],
    in_conf1: [u32; NCH],
    in_int_raw: [u32; NCH],
    in_int_ena: [u32; NCH],
    in_link: [u32; NCH],
    in_peri_sel: [u32; NCH],
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

    /// OUT channel connected peripheral id (`peri_sel.sel`, 6 bits).
    pub fn out_peri_sel(&self, ch: usize) -> u32 {
        self.out_peri_sel[ch] & 0x3F
    }

    pub fn set_out_eof_des_addr(&mut self, ch: usize, addr: u32) {
        self.out_eof_des_addr[ch] = addr;
    }

    /// Assert `out_done` + `out_eof` + `out_total_eof` for an OUT channel.
    pub fn raise_out_done(&mut self, ch: usize) {
        self.out_int_raw[ch] |= (1 << 0) | (1 << 1) | (1 << 3);
    }

    /// True if any channel has a pending (raw & enabled) interrupt.
    pub fn int_pending(&self) -> bool {
        for ch in 0..NCH {
            if self.out_int_raw[ch] & self.out_int_ena[ch] != 0 {
                return true;
            }
            if self.in_int_raw[ch] & self.in_int_ena[ch] != 0 {
                return true;
            }
        }
        false
    }

    /// Masked OUT interrupt status for a channel.
    fn out_int_st(&self, ch: usize) -> u32 {
        self.out_int_raw[ch] & self.out_int_ena[ch]
    }

    fn in_int_st(&self, ch: usize) -> u32 {
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
                0x48 => self.in_peri_sel[ch],
                _ => 0,
            }
        }
    }

    /// Write a GDMA register. Returns `Some(ch)` if the write started a TX
    /// transfer on OUT channel `ch` (i.e. `out.link.start` was set), else
    /// `None`. The caller performs the descriptor-walk copy.
    pub fn write32(&mut self, offset: u32, value: u32) -> Option<usize> {
        let off = offset & 0xFFF;
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
                        return Some(ch);
                    }
                }
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
                    // RX start would begin a peripheral->memory transfer; not
                    // modeled (no GDMA RX consumer yet), but clear the park
                    // bit to mimic the FSM leaving idle.
                    if value & (1 << 21) != 0 {
                        self.out_state[ch] &= !(1 << 1);
                    }
                }
                0x48 => self.in_peri_sel[ch] = value,
                _ => {}
            }
        }
        None
    }
}
