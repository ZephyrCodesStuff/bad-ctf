use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::Spi;

pub struct W25q16<'d> {
    spi: Spi<'d, Blocking, embassy_stm32::spi::mode::Master>,
    cs: Output<'d>,
}

impl<'d> W25q16<'d> {
    pub fn new(
        spi: Spi<'d, Blocking, embassy_stm32::spi::mode::Master>,
        cs: Output<'d>,
    ) -> Self {
        Self { spi, cs }
    }

    /// Read 3-byte JEDEC ID (Winbond W25Q16JV = [0xEF, 0x40, 0x15]).
    pub fn read_jedec_id(&mut self) -> [u8; 3] {
        let mut id = [0u8; 3];
        self.cs.set_low();
        cortex_m::asm::delay(8);
        let _ = self.spi.blocking_write(&[0x9Fu8]);
        let _ = self.spi.blocking_read(&mut id);
        self.cs.set_high();
        cortex_m::asm::delay(32);
        id
    }

    /// Read `buf.len()` bytes starting at 24-bit flash address `addr` (cmd 0x03).
    pub fn read(&mut self, addr: u32, buf: &mut [u8]) {
        let cmd = [
            0x03,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];
        self.cs.set_low();
        cortex_m::asm::delay(8);
        let _ = self.spi.blocking_write(&cmd);
        let _ = self.spi.blocking_read(buf);
        self.cs.set_high();
        cortex_m::asm::delay(32);
    }

    /// Read a length-prefixed frame `[len: u16][payload: len bytes]` in a single CS transaction.
    pub fn read_frame(&mut self, addr: u32, buf: &mut [u8]) -> usize {
        let cmd = [
            0x03,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];
        let mut len_bytes = [0u8; 2];
        self.cs.set_low();
        cortex_m::asm::delay(8);
        let _ = self.spi.blocking_write(&cmd);
        let _ = self.spi.blocking_read(&mut len_bytes);
        let n = (u16::from_le_bytes(len_bytes) as usize).min(buf.len());
        let _ = self.spi.blocking_read(&mut buf[..n]);
        self.cs.set_high();
        cortex_m::asm::delay(32);
        n
    }
}
