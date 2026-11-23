use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use embassy_stm32::gpio::{AfioRemap, OutputType};
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::peripherals::{PB1, TIM3};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Ch4;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{Peri, interrupt, pac};

/// Ultrasonic PWM carrier & audio sample rate (64 MHz / 2048 = 31.25 kHz).
const SAMPLE_RATE: u32 = 31_250;

// Phase increments for MIDI notes 120..=131 (C9..B9) at 31.25 kHz.
const OCT9_INC: [u32; 12] = [
    1_150_627_238, // C9
    1_219_046_947, // C#9
    1_291_535_236, // D9
    1_368_333_818, // D#9
    1_449_698_147, // E9
    1_535_898_373, // F9
    1_627_220_345, // F#9
    1_723_966_689, // G9
    1_826_457_949, // G#9
    1_935_033_776, // A9
    2_050_054_199, // A#9
    2_171_900_989, // B9
];

#[inline(always)]
const fn midi_to_inc(midi: u8) -> u32 {
    let m = if midi > 127 { 127 } else { midi };
    let oct = m / 12;
    let semi = (m % 12) as usize;
    let shift = 10 - oct;
    OCT9_INC[semi] >> shift
}

/// Logarithmic master volume steps (PWM duty out of ARR = 2047).
const VOL_STEPS: [u16; 10] = [0, 32, 64, 110, 180, 290, 460, 720, 1150, 1800];
const DEFAULT_VOL_IDX: u16 = 4;

static PHASE_INC: AtomicU32 = AtomicU32::new(0);
static VOL_IDX: AtomicU16 = AtomicU16::new(DEFAULT_VOL_IDX);
static DUTY_AMP: AtomicU16 = AtomicU16::new(VOL_STEPS[DEFAULT_VOL_IDX as usize]);
static PAUSED: AtomicBool = AtomicBool::new(false);

// Phase accumulator owned exclusively by the TIM3 ISR.
static mut PHASE: u32 = 0;

pub fn volume_up() -> u16 {
    let idx = (VOL_IDX.load(Ordering::Relaxed) + 1).min((VOL_STEPS.len() - 1) as u16);
    VOL_IDX.store(idx, Ordering::Relaxed);
    DUTY_AMP.store(VOL_STEPS[idx as usize], Ordering::Relaxed);
    idx
}

pub fn volume_down() -> u16 {
    let idx = VOL_IDX.load(Ordering::Relaxed).saturating_sub(1);
    VOL_IDX.store(idx, Ordering::Relaxed);
    DUTY_AMP.store(VOL_STEPS[idx as usize], Ordering::Relaxed);
    idx
}

pub fn toggle_pause() -> bool {
    let next = !PAUSED.load(Ordering::Relaxed);
    PAUSED.store(next, Ordering::Relaxed);
    next
}

pub fn is_paused() -> bool {
    PAUSED.load(Ordering::Relaxed)
}

/// Play a MIDI note (0..=127) as a 25% duty pulse wave, or silence if `midi >= 128`.
pub fn set_mono_note(midi: u8) {
    if midi < 128 {
        PHASE_INC.store(midi_to_inc(midi), Ordering::Relaxed);
    } else {
        PHASE_INC.store(0, Ordering::Relaxed);
    }
}

/// Initialize TIM3_CH4 on PB1 as a 31.25 kHz PWM DAC driven by the TIM3 update interrupt.
pub fn init_buzzer(tim3: Peri<'static, TIM3>, pb1: Peri<'static, PB1>) {
    let ch4_pin: PwmPin<'_, TIM3, Ch4, AfioRemap<0>> = PwmPin::new(pb1, OutputType::PushPull);
    let mut pwm = SimplePwm::new(
        tim3,
        None,
        None,
        None,
        Some(ch4_pin),
        Hertz(SAMPLE_RATE),
        CountingMode::EdgeAlignedUp,
    );

    let mut ch4 = pwm.ch4();
    ch4.set_duty_cycle_fully_off();
    ch4.enable();

    core::mem::forget(ch4);
    core::mem::forget(pwm);

    pac::TIM3.sr().modify(|w| w.set_uif(false));
    pac::TIM3.dier().modify(|w| w.set_uie(true));
    unsafe {
        pac::Interrupt::TIM3.enable();
    }
}

#[interrupt]
fn TIM3() {
    pac::TIM3.sr().modify(|w| w.set_uif(false));

    let inc = PHASE_INC.load(Ordering::Relaxed);
    if inc == 0 || PAUSED.load(Ordering::Relaxed) {
        pac::TIM3.ccr(3).write(|w| w.set_ccr(0));
        return;
    }

    let ph = unsafe {
        PHASE = PHASE.wrapping_add(inc);
        PHASE
    };

    // 25% duty pulse wave
    let duty = if (ph >> 30) == 0 {
        DUTY_AMP.load(Ordering::Relaxed)
    } else {
        0
    };
    pac::TIM3.ccr(3).write(|w| w.set_ccr(duty));
}
