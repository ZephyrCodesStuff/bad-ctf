#![no_std]
#![no_main]

mod display;
mod synth;
mod w25q;

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
use {defmt_rtt as _, panic_probe as _};

use crate::display::St7789;
use crate::w25q::W25q16;

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
});

// Bad Apple 160x120 1-bit framebuffer + compressed frame buffer in BSS (owned by main)
static mut VIDEO_FB: [u8; 2400] = [0; 2400];
static mut COMP_BUF: [u8; 2432] = [0; 2432];
static mut SHARED_FLASH: Option<Mutex<NoopRawMutex, W25q16<'static>>> = None;

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

    // LCD SPI1 (PA5=SCK, PA7=MOSI @ 32 MHz DMA)
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(32_000_000);
    spi_cfg.mode = spi::MODE_0;
    let spi = Spi::new_txonly(p.SPI1, p.PA5, p.PA7, p.DMA1_CH3, Irqs, spi_cfg);

    // W25Q16JVSS external SPI NOR flash on SPI2 (PB12=CS, PB13=CLK, PB14=DO/MISO, PB15=DI/MOSI)
    let mut flash_spi_cfg = spi::Config::default();
    flash_spi_cfg.frequency = Hertz(4_000_000);
    flash_spi_cfg.mode = spi::MODE_0;
    let flash_spi = Spi::new_blocking(p.SPI2, p.PB13, p.PB15, p.PB14, flash_spi_cfg);
    let flash_cs = Output::new(p.PB12, Level::High, Speed::VeryHigh);
    let mut flash = W25q16::new(flash_spi, flash_cs);

    let jedec = flash.read_jedec_id();
    info!("W25Q16 JEDEC ID: {:02x}", jedec);

    let mut hdr = [0u8; 28];
    flash.read(0, &mut hdr);

    // Buzzer PWM DAC on PB1 (TIM3_CH4)
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
    info!("ST7789T3 initialized (320x240 RGB444 @ 39 Hz)");

    if &hdr[0..4] != b"YAV1" {
        info!("No YAV1 header found on W25Q16");
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    let fps = u16::from_le_bytes([hdr[8], hdr[9]]) as u64;
    let music_us = u16::from_le_bytes([hdr[10], hdr[11]]) as u64;
    let num_frames = u32::from_le_bytes([hdr[12], hdr[13], hdr[14], hdr[15]]);
    let music_off = u32::from_le_bytes([hdr[16], hdr[17], hdr[18], hdr[19]]);
    let music_len = u32::from_le_bytes([hdr[20], hdr[21], hdr[22], hdr[23]]);
    let video_off = u32::from_le_bytes([hdr[24], hdr[25], hdr[26], hdr[27]]);
    info!(
        "YAV1 Bad Apple: {} frames @ {} fps, music {} B ({} us/tick)",
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

    #[allow(static_mut_refs)]
    let (fb, comp) = unsafe { (&mut VIDEO_FB, &mut COMP_BUF) };

    loop {
        fb.fill(0);
        lcd.clear_black().await;
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
