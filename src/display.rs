use core::convert::Infallible;

use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_time::{Duration, Timer};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;

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

    /// Init sequence matching the stock firmware for HS20HS072RX (ST7789T3, 320x240 landscape).
    pub async fn init(&mut self) {
        self.cs.set_high();
        self.rst.set_low();
        Timer::after(Duration::from_millis(10)).await;
        self.rst.set_high();
        Timer::after(Duration::from_millis(120)).await;

        // MADCTL: MY=1, MX=0, MV=1 -> 320x240 landscape
        self.write_cmd(0x36, &[0xA0]).await;
        // COLMOD: 16-bit RGB565
        self.write_cmd(0x3A, &[0x05]).await;
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
        // Frame Rate Control (60 Hz)
        self.write_cmd(0xC6, &[0x0F]).await;
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

        self.clear(0x0000).await;

        // Normal Display Mode On & Display On
        self.write_cmd(0x13, &[]).await;
        self.write_cmd(0x29, &[]).await;
    }

    pub async fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
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

    pub async fn fill_rect(&mut self, x: u16, y: u16, w: u16, h: u16, rgb565: u16) {
        if w == 0 || h == 0 {
            return;
        }
        let x1 = (x + w - 1).min(WIDTH - 1);
        let y1 = (y + h - 1).min(HEIGHT - 1);
        self.set_window(x, y, x1, y1).await;

        let hi = (rgb565 >> 8) as u8;
        let lo = (rgb565 & 0xFF) as u8;
        let mut buf = [0u8; 320];
        for chunk in buf.chunks_exact_mut(2) {
            chunk[0] = hi;
            chunk[1] = lo;
        }

        let mut remaining_bytes = ((x1 - x + 1) as usize) * ((y1 - y + 1) as usize) * 2;

        self.cs.set_low();
        self.dc.set_low();
        let _ = self.spi.write(&[0x2Cu8]).await;
        self.dc.set_high();

        while remaining_bytes > 0 {
            let n = remaining_bytes.min(buf.len());
            let _ = self.spi.write(&buf[..n]).await;
            remaining_bytes -= n;
        }

        self.cs.set_high();
    }

    pub async fn clear(&mut self, rgb565: u16) {
        self.fill_rect(0, 0, WIDTH, HEIGHT, rgb565).await;
    }

    /// Draw a 1-bit MSB-first packed bitmap mask (`w` must be <= 320 and a multiple of 8).
    pub async fn draw_bitmap_1bit(
        &mut self,
        x: u16,
        y: u16,
        w: u16,
        h: u16,
        mask: &[u8],
        fg: u16,
        bg: u16,
    ) {
        let stride = (w as usize) / 8;
        let x1 = x + w - 1;
        let y1 = y + h - 1;
        self.set_window(x, y, x1, y1).await;

        let fg_hi = (fg >> 8) as u8;
        let fg_lo = (fg & 0xFF) as u8;
        let bg_hi = (bg >> 8) as u8;
        let bg_lo = (bg & 0xFF) as u8;

        let mut row_buf = [0u8; 640]; // up to 320 px wide
        let row_bytes = (w as usize) * 2;

        self.cs.set_low();
        self.dc.set_low();
        let _ = self.spi.write(&[0x2Cu8]).await;
        self.dc.set_high();

        for row in 0..(h as usize) {
            let mask_row = &mask[row * stride..(row + 1) * stride];
            let mut out = 0usize;
            for &byte in mask_row {
                for bit in (0..8).rev() {
                    if (byte & (1 << bit)) != 0 {
                        row_buf[out] = fg_hi;
                        row_buf[out + 1] = fg_lo;
                    } else {
                        row_buf[out] = bg_hi;
                        row_buf[out + 1] = bg_lo;
                    }
                    out += 2;
                }
            }
            let _ = self.spi.write(&row_buf[..row_bytes]).await;
        }

        self.cs.set_high();
    }

    /// Blit rows `min_row..=max_row` of a 160x120 1-bit framebuffer scaled 2x to 320x240.
    pub async fn draw_bitmap_1bit_2x_rows(
        &mut self,
        fb_160x120: &[u8; 2400],
        min_row: usize,
        max_row: usize,
        fg: u16,
        bg: u16,
    ) {
        if min_row > max_row || max_row >= 120 {
            return;
        }
        let y0 = (min_row * 2) as u16;
        let y1 = (max_row * 2 + 1) as u16;
        self.set_window(0, y0, WIDTH - 1, y1).await;

        let fg_hi = (fg >> 8) as u8;
        let fg_lo = (fg & 0xFF) as u8;
        let bg_hi = (bg >> 8) as u8;
        let bg_lo = (bg & 0xFF) as u8;

        // 2 scaled scanlines (320 * 2 * 2 = 1280 bytes) per DMA transfer
        let mut two_rows = [0u8; 1280];

        self.cs.set_low();
        self.dc.set_low();
        let _ = self.spi.write(&[0x2Cu8]).await;
        self.dc.set_high();

        for row in min_row..=max_row {
            let src = &fb_160x120[row * 20..(row + 1) * 20];
            let mut out = 0usize;
            for &byte in src {
                for bit in (0..8).rev() {
                    let (hi, lo) = if (byte & (1 << bit)) != 0 {
                        (fg_hi, fg_lo)
                    } else {
                        (bg_hi, bg_lo)
                    };
                    two_rows[out] = hi;
                    two_rows[out + 1] = lo;
                    two_rows[out + 2] = hi;
                    two_rows[out + 3] = lo;
                    out += 4;
                }
            }
            two_rows.copy_within(0..640, 640);
            let _ = self.spi.write(&two_rows).await;
        }

        self.cs.set_high();
    }

    /// Switch ST7789 to 12-bit RGB444 (25% faster SPI transfer) and 39 Hz refresh to minimize tearing.
    pub async fn enter_fast_bw_mode(&mut self) {
        self.write_cmd(0x3A, &[0x03]).await; // COLMOD = 12-bit/pixel (RGB444)
        self.write_cmd(0xC6, &[0x1F]).await; // FRCTRL2 = 39 Hz (reduces scanout tear crossings)
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

        // 1 scaled scanline = 320 px * 1.5 B/px = 480 bytes; 2 scanlines = 960 bytes
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

pub const fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    (((r & 0xF8) as u16) << 8) | (((g & 0xFC) as u16) << 3) | ((b as u16) >> 3)
}

/// Reusable 1-bit `embedded-graphics` canvas (320 px wide, up to 108 px tall = 4,320 bytes).
pub const CANVAS_W: usize = 320;
pub const CANVAS_MAX_H: usize = 108;
pub const CANVAS_STRIDE: usize = CANVAS_W / 8;

pub struct Canvas1Bit {
    buf: [u8; CANVAS_STRIDE * CANVAS_MAX_H],
    height: usize,
}

impl Canvas1Bit {
    pub const fn new() -> Self {
        Self {
            buf: [0; CANVAS_STRIDE * CANVAS_MAX_H],
            height: CANVAS_MAX_H,
        }
    }

    pub fn begin(&mut self, height: usize) {
        let h = height.min(CANVAS_MAX_H);
        self.height = h;
        self.buf[..h * CANVAS_STRIDE].fill(0);
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.height * CANVAS_STRIDE]
    }
}

impl OriginDimensions for Canvas1Bit {
    fn size(&self) -> Size {
        Size::new(CANVAS_W as u32, self.height as u32)
    }
}

impl DrawTarget for Canvas1Bit {
    type Color = BinaryColor;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(Point { x, y }, color) in pixels {
            if x >= 0 && y >= 0 && (x as usize) < CANVAS_W && (y as usize) < self.height {
                let idx = (y as usize) * CANVAS_STRIDE + ((x as usize) >> 3);
                let mask = 1u8 << (7 - (x as usize & 7));
                if color == BinaryColor::On {
                    self.buf[idx] |= mask;
                } else {
                    self.buf[idx] &= !mask;
                }
            }
        }
        Ok(())
    }
}
