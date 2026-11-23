use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_time::{Duration, Timer};

pub const WIDTH: u16 = 320;
pub const HEIGHT: u16 = 240;

pub struct St7789<'d> {
    spi: Spi<'d, Async, embassy_stm32::spi::mode::Master>,
    cs: Output<'d>,
    dc: Output<'d>,
    rst: Output<'d>,
}

impl<'d> St7789<'d> {
    pub fn new(
        spi: Spi<'d, Async, embassy_stm32::spi::mode::Master>,
        cs: Output<'d>,
        dc: Output<'d>,
        rst: Output<'d>,
    ) -> Self {
        Self { spi, cs, dc, rst }
    }

    async fn write_cmd(&mut self, cmd: u8, data: &[u8]) {
        self.cs.set_low();

        self.dc.set_low();
        let _ = self.spi.write(&[cmd]).await;

        if !data.is_empty() {
            self.dc.set_high();
            let _ = self.spi.write(data).await;
        }

        self.cs.set_high();
    }

    /// Init sequence for HS20HS072RX (ST7789T3, 320x240 landscape, 12-bit RGB444, 39 Hz).
    pub async fn init(&mut self) {
        self.cs.set_high();
        self.rst.set_low();
        Timer::after(Duration::from_millis(10)).await;
        self.rst.set_high();
        Timer::after(Duration::from_millis(120)).await;

        // MADCTL: MY=1, MX=0, MV=1 -> 320x240 landscape
        self.write_cmd(0x36, &[0xA0]).await;
        // COLMOD: 12-bit RGB444 (25% faster SPI transfer for B/W video)
        self.write_cmd(0x3A, &[0x03]).await;
        // Porch Setting
        self.write_cmd(0xB2, &[0x0C, 0x0C, 0x00, 0x33, 0x33]).await;
        // Gate Control
        self.write_cmd(0xB7, &[0x35]).await;
        // VCOM Setting
        self.write_cmd(0xBB, &[0x20]).await;
        // LCM Control
        self.write_cmd(0xC0, &[0x2C]).await;
        // VDV and VRH Command Enable
        self.write_cmd(0xC2, &[0x01]).await;
        // VRH Set
        self.write_cmd(0xC3, &[0x0B]).await;
        // VDV Set
        self.write_cmd(0xC4, &[0x20]).await;
        // Frame Rate Control: 39 Hz (reduces scanout tear crossings)
        self.write_cmd(0xC6, &[0x1F]).await;
        // Power Control 1
        self.write_cmd(0xD0, &[0xA4, 0xA1]).await;
        // Positive Voltage Gamma Control
        self.write_cmd(
            0xE0,
            &[
                0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23,
            ],
        )
        .await;
        // Negative Voltage Gamma Control
        self.write_cmd(
            0xE1,
            &[
                0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23,
            ],
        )
        .await;
        // Display Inversion ON (IPS panel)
        self.write_cmd(0x21, &[]).await;
        // Sleep Out
        self.write_cmd(0x11, &[]).await;
        Timer::after(Duration::from_millis(120)).await;

        self.clear_black().await;

        // Normal Display Mode On & Display On
        self.write_cmd(0x13, &[]).await;
        self.write_cmd(0x29, &[]).await;
    }

    async fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        let col = [
            (x0 >> 8) as u8,
            (x0 & 0xFF) as u8,
            (x1 >> 8) as u8,
            (x1 & 0xFF) as u8,
        ];
        let row = [
            (y0 >> 8) as u8,
            (y0 & 0xFF) as u8,
            (y1 >> 8) as u8,
            (y1 & 0xFF) as u8,
        ];
        self.write_cmd(0x2A, &col).await;
        self.write_cmd(0x2B, &row).await;
    }

    pub async fn clear_black(&mut self) {
        self.set_window(0, 0, WIDTH - 1, HEIGHT - 1).await;
        let zeros = [0u8; 960]; // 2 scanlines in 12-bit RGB444

        self.cs.set_low();
        self.dc.set_low();
        let _ = self.spi.write(&[0x2Cu8]).await;
        self.dc.set_high();

        for _ in 0..120 {
            let _ = self.spi.write(&zeros).await;
        }

        self.cs.set_high();
    }

    /// Blit rows `min_row..=max_row` of a 160x120 1-bit framebuffer scaled 2x to 320x240 in 12-bit RGB444.
    /// Each 1-bit source pixel becomes 2 horizontal pixels = 3 bytes (0x00_00_00 or 0xFF_FF_FF).
    pub async fn draw_bitmap_1bit_2x_rows_bw(
        &mut self,
        fb_160x120: &[u8; 2400],
        min_row: usize,
        max_row: usize,
    ) {
        if min_row > max_row || max_row >= 120 {
            return;
        }
        let y0 = (min_row * 2) as u16;
        let y1 = (max_row * 2 + 1) as u16;
        self.set_window(0, y0, WIDTH - 1, y1).await;

        let mut two_rows = [0u8; 960];

        self.cs.set_low();
        self.dc.set_low();
        let _ = self.spi.write(&[0x2Cu8]).await;
        self.dc.set_high();

        for row in min_row..=max_row {
            let src = &fb_160x120[row * 20..(row + 1) * 20];
            let mut out = 0usize;
            for &byte in src {
                for bit in (0..8).rev() {
                    let v = if (byte & (1 << bit)) != 0 { 0xFF } else { 0x00 };
                    two_rows[out] = v;
                    two_rows[out + 1] = v;
                    two_rows[out + 2] = v;
                    out += 3;
                }
            }
            two_rows.copy_within(0..480, 480);
            let _ = self.spi.write(&two_rows).await;
        }

        self.cs.set_high();
    }
}
