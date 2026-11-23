#![no_std]
#![no_main]

mod display;
mod song_data;
mod synth;
mod w25q;

use core::fmt::Write;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::rcc::{APBPrescaler, Pll, PllMul, PllPreDiv, PllSource, Sysclk};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, bind_interrupts, dma, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_7X13_BOLD};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;
use {defmt_rtt as _, panic_probe as _};

use crate::display::{CANVAS_W, Canvas1Bit, St7789, rgb565};
use crate::synth::{SCOPE_LEN, VoiceInfo};
use crate::w25q::W25q16;

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
});

// Reusable 4.3 KiB 1-bit canvas in BSS (owned exclusively by main)
static mut CANVAS: Canvas1Bit = Canvas1Bit::new();
// Bad Apple 160x120 1-bit framebuffer + compressed frame buffer in BSS (owned by main)
static mut VIDEO_FB: [u8; 2400] = [0; 2400];
static mut COMP_BUF: [u8; 2432] = [0; 2432];
static mut SHARED_FLASH: Option<Mutex<NoopRawMutex, W25q16<'static>>> = None;

struct StrBuf<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> StrBuf<N> {
    const fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
        }
    }

    fn clear(&mut self) {
        self.len = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl<const N: usize> Write for StrBuf<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let avail = N - self.len;
        let n = bytes.len().min(avail);
        self.buf[self.len..self.len + n].copy_from_slice(&bytes[..n]);
        self.len += n;
        Ok(())
    }
}

const NOTE_NAMES: [&str; 12] = [
    "C ", "C#", "D ", "D#", "E ", "F ", "F#", "G ", "G#", "A ", "A#", "B ",
];

fn wave_tag(v: &VoiceInfo) -> &'static str {
    if v.is_noise {
        return "NOI";
    }
    match v.wave {
        0 => "1/8",
        1 => "1/4",
        2 => "SQR",
        3 => "SAW",
        4 => "2SW",
        _ => "WAV",
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 64 MHz SYSCLK from internal HSI (8 MHz / 2 * 16), APB1 = 32 MHz, APB2 = 64 MHz
    let mut cfg = Config::default();
    cfg.rcc.hsi = true;
    cfg.rcc.pll = Some(Pll {
        src: PllSource::HSI,
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL16,
    });
    cfg.rcc.sys = Sysclk::PLL1_P;
    cfg.rcc.apb1_pre = APBPrescaler::DIV2;
    cfg.rcc.apb2_pre = APBPrescaler::DIV1;

    let p = embassy_stm32::init(cfg);
    info!("ybox-cfw booting (64 MHz HSI+PLL)");

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(32_000_000);
    spi_cfg.mode = spi::MODE_0;

    let spi = Spi::new_txonly(p.SPI1, p.PA5, p.PA7, p.DMA1_CH3, Irqs, spi_cfg);

    // W25Q16JVSS external SPI NOR flash on SPI2 (PB12=CS, PB13=CLK, PB14=DO/MISO, PB15=DI/MOSI)
    let mut flash_spi_cfg = spi::Config::default();
    flash_spi_cfg.frequency = Hertz(4_000_000);
    flash_spi_cfg.mode = spi::MODE_0;
    let flash_spi = Spi::new_blocking(
        p.SPI2,
        p.PB13,
        p.PB15,
        p.PB14,
        flash_spi_cfg,
    );
    let flash_cs = Output::new(p.PB12, Level::High, Speed::VeryHigh);
    let mut flash = W25q16::new(flash_spi, flash_cs);

    let jedec = flash.read_jedec_id();
    info!("W25Q16 JEDEC ID: {:02x}", jedec);

    let mut hdr = [0u8; 28];
    flash.read(0, &mut hdr);

    // Initialize buzzer PWM DAC on PB1 (TIM3_CH4)
    synth::init_buzzer(p.TIM3, p.PB1);

    // Buttons: PB8 = UP, PB9 = DOWN, PA8 = B (pause/resume), active-low to GND
    let btn_up = Input::new(p.PB8, Pull::Up);
    let btn_down = Input::new(p.PB9, Pull::Up);
    let btn_b = Input::new(p.PA8, Pull::Up);
    spawner.spawn(buttons_task(btn_up, btn_down, btn_b).unwrap());

    // Ensure SWD stays enabled after SPI1, SPI2 & TIM3 AFIO.MAPR remap writes.
    embassy_stm32::pac::AFIO.mapr().modify(|w| {
        w.set_swj_cfg(embassy_stm32::pac::afio::vals::SwjCfg::JTAG_DISABLE);
    });

    let cs = Output::new(p.PA2, Level::High, Speed::VeryHigh);
    let dc = Output::new(p.PA3, Level::Low, Speed::VeryHigh);
    let rst = Output::new(p.PA4, Level::High, Speed::VeryHigh);

    let mut lcd = St7789::new(spi, cs, dc, rst);
    lcd.init().await;
    info!("ST7789T3 initialized (320x240)");

    if &hdr[0..4] == b"YAV1" {
        let fps = u16::from_le_bytes([hdr[8], hdr[9]]) as u64;
        let music_us = u16::from_le_bytes([hdr[10], hdr[11]]) as u64;
        let num_frames = u32::from_le_bytes([hdr[12], hdr[13], hdr[14], hdr[15]]);
        let music_off = u32::from_le_bytes([hdr[16], hdr[17], hdr[18], hdr[19]]);
        let music_len = u32::from_le_bytes([hdr[20], hdr[21], hdr[22], hdr[23]]);
        let video_off = u32::from_le_bytes([hdr[24], hdr[25], hdr[26], hdr[27]]);
        info!(
            "YAV1 Bad Apple found: {} frames @ {} fps, music {} B ({} us/tick)",
            num_frames, fps, music_len, music_us
        );

        #[allow(static_mut_refs)]
        let flash_ref: &'static Mutex<NoopRawMutex, W25q16<'static>> = unsafe {
            SHARED_FLASH = Some(Mutex::new(flash));
            SHARED_FLASH.as_ref().unwrap()
        };

        spawner.spawn(yav_music_task(flash_ref, music_off, music_len, music_us).unwrap());

        // Wait 0.5s in sync with yav_music_task
        Timer::after(Duration::from_millis(500)).await;

        let frame_dur = Duration::from_micros(1_000_000 / fps.max(1));
        lcd.enter_fast_bw_mode().await;

        #[allow(static_mut_refs)]
        let (fb, comp) = unsafe { (&mut VIDEO_FB, &mut COMP_BUF) };

        loop {
            fb.fill(0);
            let mut addr = video_off;
            let mut next_at = Instant::now();

            for _ in 0..num_frames {
                while synth::is_paused() {
                    Timer::after(Duration::from_millis(20)).await;
                    next_at = Instant::now();
                }

                let comp_len = {
                    let mut f = flash_ref.lock().await;
                    f.read_frame(addr, comp)
                };
                addr += 2 + comp_len as u32;

                // Decode PackBits-RLE XOR diff directly into 160x120 framebuffer, tracking dirty rows
                if let Some((min_row, max_row)) = packbits_xor_decode(&comp[..comp_len], fb) {
                    lcd.draw_bitmap_1bit_2x_rows_bw(fb, min_row, max_row).await;
                }

                next_at += frame_dur;
                let now = Instant::now();
                if next_at > now {
                    Timer::at(next_at).await;
                } else {
                    next_at = now;
                }
            }
        }
    } else if &hdr[0..4] == b"YAD1" {
        let sample_rate = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
        let num_samples = u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]);
        info!("W25Q16 YAD1 audio found: {} Hz, {} samples", sample_rate, num_samples);
        synth::enable_adpcm_mode(num_samples);
        spawner.spawn(flash_audio_task(flash, num_samples).unwrap());
    } else {
        info!("No YAV1/YAD1 header on W25Q16, playing built-in chiptune");
        spawner.spawn(synth::chiptune_task().unwrap());
    }

    let header_style = MonoTextStyle::new(&FONT_7X13_BOLD, BinaryColor::On);
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let fill = PrimitiveStyle::with_fill(BinaryColor::On);

    let col_header_fg = rgb565(120, 230, 255);
    let col_header_bg = rgb565(12, 24, 48);
    let col_scope_fg = rgb565(64, 255, 110);
    let col_voices_fg = rgb565(255, 210, 90);
    let col_black = rgb565(0, 0, 0);

    let mut scope = [0u8; SCOPE_LEN];
    let mut line_buf = StrBuf::<48>::new();

    #[allow(static_mut_refs)]
    let canvas = unsafe { &mut CANVAS };

    loop {
        synth::read_scope(&mut scope);
        let snap = synth::snapshot();

        // 1) Header band (320x24 at y = 0)
        canvas.begin(24);
        line_buf.clear();
        if snap.adpcm_mode {
            let cur_s = snap.adpcm_played / 15_625;
            let tot_s = snap.adpcm_total / 15_625;
            let _ = write!(
                line_buf,
                "{} {:02}:{:02}/{:02}:{:02}  ADPCM  VOL:{}/{}",
                if snap.paused { "PAUSE" } else { "FLASH" },
                cur_s / 60,
                cur_s % 60,
                tot_s / 60,
                tot_s % 60,
                snap.vol_idx,
                snap.max_vol_idx
            );
        } else {
            let _ = write!(
                line_buf,
                "{} BAR {:02}/{:02}  V:{:02}  VOL:{}/{}",
                if snap.paused { "PAUSE" } else { "ZBOX " },
                snap.bar + 1,
                snap.total_bars,
                snap.active_count,
                snap.vol_idx,
                snap.max_vol_idx
            );
        }
        let _ = Text::new(line_buf.as_str(), Point::new(8, 16), header_style).draw(canvas);
        lcd.draw_bitmap_1bit(0, 0, CANVAS_W as u16, 24, canvas.as_slice(), col_header_fg, col_header_bg)
            .await;

        // 2) Live oscilloscope band (320x96 at y = 26)
        canvas.begin(96);
        let _ = Rectangle::new(Point::new(0, 0), Size::new(320, 96))
            .into_styled(stroke)
            .draw(canvas);
        // Dotted center baseline
        for x in (4..316).step_by(6) {
            let _ = Pixel(Point::new(x, 48), BinaryColor::On).draw(canvas);
        }
        // Plot 160 samples stretched 2px wide across x = 2..318, y = 91..4
        let mut prev_y = 91 - ((scope[0] as i32 * 86) >> 8);
        for i in 1..SCOPE_LEN {
            let y = 91 - ((scope[i] as i32 * 86) >> 8);
            let x0 = (i as i32 - 1) * 2 + 1;
            let x1 = (i as i32) * 2 + 1;
            let _ = Line::new(Point::new(x0, prev_y), Point::new(x1, y))
                .into_styled(stroke)
                .draw(canvas);
            prev_y = y;
        }
        lcd.draw_bitmap_1bit(0, 26, CANVAS_W as u16, 96, canvas.as_slice(), col_scope_fg, col_black)
            .await;

        // 3) Bottom band (320x108 at y = 128)
        canvas.begin(108);
        if snap.adpcm_mode {
            let _ = Text::new("W25Q16 SPI FLASH STREAM", Point::new(6, 14), header_style).draw(canvas);
            let _ = Line::new(Point::new(6, 20), Point::new(313, 20))
                .into_styled(stroke)
                .draw(canvas);
            let _ = Text::new("4-bit IMA ADPCM @ 15.625 kHz", Point::new(6, 38), text_style).draw(canvas);

            // Progress bar
            let _ = Rectangle::new(Point::new(6, 54), Size::new(308, 16))
                .into_styled(stroke)
                .draw(canvas);
            let fill_w = if snap.adpcm_total > 0 {
                ((snap.adpcm_played.min(snap.adpcm_total) as u64 * 304) / (snap.adpcm_total as u64)) as u32
            } else {
                0
            };
            if fill_w > 0 {
                let _ = Rectangle::new(Point::new(8, 56), Size::new(fill_w, 12))
                    .into_styled(fill)
                    .draw(canvas);
            }

            let _ = Text::new("UP/DN: Volume   B: Pause/Resume", Point::new(6, 92), text_style).draw(canvas);
        } else {
            let _ = Text::new("ACTIVE VOICES", Point::new(6, 10), text_style).draw(canvas);
            let _ = Line::new(Point::new(6, 14), Point::new(313, 14))
                .into_styled(stroke)
                .draw(canvas);

            let mut shown = 0usize;
            for (idx, v) in snap.voices.iter().enumerate() {
                if !v.active {
                    continue;
                }
                if shown >= 10 {
                    break;
                }
                let col = (shown / 5) as i32;
                let row = (shown % 5) as i32;
                let bx = 6 + col * 158;
                let by = 28 + row * 17;

                line_buf.clear();
                if v.is_noise {
                    let _ = write!(line_buf, "D{} NOI P{:02}", idx - 8, v.midi);
                } else {
                    let oct = (v.midi as i32 / 12) - 1;
                    let note = NOTE_NAMES[(v.midi % 12) as usize];
                    let _ = write!(line_buf, "V{} {} {}{}", idx, wave_tag(v), note, oct);
                }
                let _ = Text::new(line_buf.as_str(), Point::new(bx, by), text_style).draw(canvas);

                // Mini level meter bar beside each voice
                let bar_w = ((v.vol as u32 * 44) / 80).clamp(2, 44);
                let _ = Rectangle::new(Point::new(bx + 104, by - 8), Size::new(bar_w, 8))
                    .into_styled(fill)
                    .draw(canvas);

                shown += 1;
            }

            if shown == 0 {
                let _ = Text::new("(silence)", Point::new(6, 30), text_style).draw(canvas);
            }
        }

        lcd.draw_bitmap_1bit(0, 128, CANVAS_W as u16, 108, canvas.as_slice(), col_voices_fg, col_black)
            .await;

        Timer::after(Duration::from_millis(33)).await;
    }
}

#[embassy_executor::task]
async fn buttons_task(up: Input<'static>, down: Input<'static>, btn_b: Input<'static>) {
    let mut prev_up = up.is_high();
    let mut prev_down = down.is_high();
    let mut prev_b = btn_b.is_high();

    loop {
        Timer::after(Duration::from_millis(20)).await;

        let cur_up = up.is_high();
        if prev_up && !cur_up {
            let _ = synth::volume_up();
        }
        prev_up = cur_up;

        let cur_down = down.is_high();
        if prev_down && !cur_down {
            let _ = synth::volume_down();
        }
        prev_down = cur_down;

        let cur_b = btn_b.is_high();
        if prev_b && !cur_b {
            let _ = synth::toggle_pause();
        }
        prev_b = cur_b;
    }
}

fn packbits_xor_decode(src: &[u8], dst: &mut [u8]) -> Option<(usize, usize)> {
    let mut si = 0usize;
    let mut di = 0usize;
    let mut min_byte = usize::MAX;
    let mut max_byte = 0usize;

    while si < src.len() && di < dst.len() {
        let ctrl = src[si];
        si += 1;
        if ctrl < 128 {
            let count = (ctrl as usize) + 1;
            for _ in 0..count {
                if si >= src.len() || di >= dst.len() {
                    break;
                }
                let val = src[si];
                si += 1;
                if val != 0 {
                    dst[di] ^= val;
                    if di < min_byte {
                        min_byte = di;
                    }
                    if di > max_byte {
                        max_byte = di;
                    }
                }
                di += 1;
            }
        } else {
            let count = 257usize - (ctrl as usize);
            if si >= src.len() {
                break;
            }
            let val = src[si];
            si += 1;
            if val == 0 {
                di = (di + count).min(dst.len());
            } else {
                let end = (di + count).min(dst.len());
                if di < end {
                    if di < min_byte {
                        min_byte = di;
                    }
                    if end - 1 > max_byte {
                        max_byte = end - 1;
                    }
                    for b in &mut dst[di..end] {
                        *b ^= val;
                    }
                    di = end;
                }
            }
        }
    }

    if min_byte <= max_byte {
        Some((min_byte / 20, max_byte / 20))
    } else {
        None
    }
}

#[embassy_executor::task]
async fn yav_music_task(
    flash: &'static Mutex<NoopRawMutex, W25q16<'static>>,
    music_off: u32,
    music_len: u32,
    music_us: u64,
) {
    Timer::after(Duration::from_millis(500)).await;

    let mut buf = [0u8; 128];
    loop {
        let mut pos: u32 = 0;
        let mut next_at = Instant::now();

        while pos < music_len {
            let n = ((music_len - pos) as usize).min(buf.len()) & !1;
            if n == 0 {
                break;
            }
            {
                let mut f = flash.lock().await;
                f.read(music_off + pos, &mut buf[..n]);
            }
            pos += n as u32;

            for pair in buf[..n].chunks_exact(2) {
                while synth::is_paused() {
                    synth::set_mono_note(0xFE);
                    Timer::after(Duration::from_millis(20)).await;
                    next_at = Instant::now();
                }

                let note = pair[0];
                let dur = pair[1] as u64;
                synth::set_mono_note(note);

                next_at += Duration::from_micros(dur * music_us);
                let now = Instant::now();
                if next_at > now {
                    Timer::at(next_at).await;
                } else {
                    next_at = now;
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn flash_audio_task(mut flash: W25q16<'static>, num_samples: u32) {
    // Wait 0.5s after startup before playing.
    Timer::after(Duration::from_millis(500)).await;

    let total_bytes = (num_samples + 1) / 2;
    let mut offset: u32 = 0;
    let mut chunk = [0u8; 256];

    loop {
        if synth::adpcm_free_space() >= chunk.len() {
            let rem = total_bytes - offset;
            let n = (rem as usize).min(chunk.len());
            flash.read(16 + offset, &mut chunk[..n]);
            synth::adpcm_push(&chunk[..n]);
            offset += n as u32;
            if offset >= total_bytes {
                offset = 0;
                synth::adpcm_request_reset();
            }
        } else {
            Timer::after(Duration::from_millis(8)).await;
        }
    }
}
