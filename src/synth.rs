use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, AtomicUsize, Ordering};

use embassy_stm32::gpio::{AfioRemap, OutputType};
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::peripherals::{PB1, TIM3};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Ch4;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{Peri, interrupt, pac};
use embassy_time::{Duration, Timer};

use crate::song_data::{
    CHANNELS, HALF_TICK_US, HALF_TICKS_PER_BAR, INTRO_BARS, NOTES, NUM_BARS, NUM_CHANNELS,
    PAT_RANGES, SEQUENCE,
};

/// Ultrasonic PWM carrier & audio sample rate (64 MHz / 2048 = 31.25 kHz).
const SAMPLE_RATE: u32 = 31_250;

pub const NUM_PITCH_VOICES: usize = 8;
pub const NUM_NOISE_VOICES: usize = 2;
pub const SCOPE_LEN: usize = 160;

// Phase increments for MIDI notes 120..=131 (C9..B9) at 31.25 kHz:
// inc = round(freq_hz * 2^32 / 31250).
const OCT9_INC: [u32; 12] = [
    1_150_627_238, // C9  (8372.02 Hz)
    1_219_046_947, // C#9
    1_291_535_236, // D9
    1_368_333_818, // D#9
    1_449_698_147, // E9
    1_535_898_373, // F9
    1_627_220_345, // F#9
    1_723_966_689, // G9
    1_826_457_949, // G#9
    1_935_033_776, // A9  (14080.00 Hz)
    2_050_054_199, // A#9
    2_171_900_989, // B9
];

#[inline(always)]
const fn midi_to_inc(midi: u8) -> u32 {
    let m = if midi > 127 { 127 } else { midi };
    let oct = m / 12; // 0..=10
    let semi = (m % 12) as usize;
    let shift = 10 - oct; // 10..=0
    OCT9_INC[semi] >> shift
}

// Lock-free pitch voice state read by TIM3 ISR & UI snapshot
static P_INC: [AtomicU32; NUM_PITCH_VOICES] = [const { AtomicU32::new(0) }; NUM_PITCH_VOICES];
static P_VOL: [AtomicU16; NUM_PITCH_VOICES] = [const { AtomicU16::new(0) }; NUM_PITCH_VOICES];
static P_WAVE: [AtomicU8; NUM_PITCH_VOICES] = [const { AtomicU8::new(0) }; NUM_PITCH_VOICES];
static P_MIDI: [AtomicU8; NUM_PITCH_VOICES] = [const { AtomicU8::new(0) }; NUM_PITCH_VOICES];

// Lock-free noise (drum) voice state read by TIM3 ISR & UI snapshot
static N_INC: [AtomicU32; NUM_NOISE_VOICES] = [const { AtomicU32::new(0) }; NUM_NOISE_VOICES];
static N_VOL: [AtomicU16; NUM_NOISE_VOICES] = [const { AtomicU16::new(0) }; NUM_NOISE_VOICES];
static N_MIDI: [AtomicU8; NUM_NOISE_VOICES] = [const { AtomicU8::new(0) }; NUM_NOISE_VOICES];

static CUR_BAR: AtomicU16 = AtomicU16::new(0);
static PAUSED: AtomicBool = AtomicBool::new(false);

// --- Flash IMA-ADPCM streaming state ---
const RING_SIZE: usize = 2048;
const RING_MASK: usize = RING_SIZE - 1;
static ADPCM_MODE: AtomicBool = AtomicBool::new(false);
static ADPCM_RESET: AtomicBool = AtomicBool::new(false);
static ADPCM_TOTAL_SAMPLES: AtomicU32 = AtomicU32::new(0);
static ADPCM_PLAYED_SAMPLES: AtomicU32 = AtomicU32::new(0);
static RING_HEAD: AtomicUsize = AtomicUsize::new(0); // written by producer (flash task)
static RING_TAIL: AtomicUsize = AtomicUsize::new(0); // written by consumer (TIM3 ISR)
static mut AUDIO_RING: [u8; RING_SIZE] = [0; RING_SIZE];

// ADPCM decoder state owned exclusively by TIM3 ISR
static mut ADPCM_DIV: u8 = 0;
static mut ADPCM_HAVE_HI: bool = false;
static mut ADPCM_CUR_BYTE: u8 = 0;
static mut ADPCM_PRED: i32 = 0;
static mut ADPCM_STEP_IDX: i8 = 0;
static mut ADPCM_LAST_SAMPLE: u8 = 128;
static mut ADPCM_PREV_SAMPLE: u8 = 128;

const IMA_INDEX_TABLE: [i8; 16] = [
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
];

const IMA_STEP_TABLE: [i32; 89] = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143,
    157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544,
    598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878,
    2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894,
    6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818,
    18500, 20350, 22385, 24623, 27086, 29794, 32767,
];

pub fn enable_adpcm_mode(total_samples: u32) {
    ADPCM_TOTAL_SAMPLES.store(total_samples, Ordering::Relaxed);
    ADPCM_PLAYED_SAMPLES.store(0, Ordering::Relaxed);
    RING_HEAD.store(0, Ordering::Relaxed);
    RING_TAIL.store(0, Ordering::Relaxed);
    ADPCM_RESET.store(true, Ordering::Release);
    ADPCM_MODE.store(true, Ordering::Release);
}

pub fn adpcm_request_reset() {
    ADPCM_RESET.store(true, Ordering::Release);
    ADPCM_PLAYED_SAMPLES.store(0, Ordering::Relaxed);
}

pub fn adpcm_free_space() -> usize {
    let head = RING_HEAD.load(Ordering::Relaxed);
    let tail = RING_TAIL.load(Ordering::Acquire);
    let used = head.wrapping_sub(tail);
    RING_SIZE.saturating_sub(1).saturating_sub(used)
}

pub fn adpcm_push(data: &[u8]) -> usize {
    let free = adpcm_free_space();
    let n = data.len().min(free);
    let mut head = RING_HEAD.load(Ordering::Relaxed);
    for &b in &data[..n] {
        unsafe {
            AUDIO_RING[head & RING_MASK] = b;
        }
        head = head.wrapping_add(1);
    }
    RING_HEAD.store(head, Ordering::Release);
    n
}

pub fn toggle_pause() -> bool {
    let next = !PAUSED.load(Ordering::Relaxed);
    PAUSED.store(next, Ordering::Relaxed);
    next
}

pub fn is_paused() -> bool {
    PAUSED.load(Ordering::Relaxed)
}

/// Play a single monophonic chiptune note (0..=127) on voice 0, or silence if `midi >= 128`.
pub fn set_mono_note(midi: u8) {
    if midi < 128 {
        P_WAVE[0].store(1, Ordering::Relaxed); // 25% duty pulse
        P_MIDI[0].store(midi, Ordering::Relaxed);
        P_INC[0].store(midi_to_inc(midi), Ordering::Relaxed);
        P_VOL[0].store(180, Ordering::Relaxed);
    } else {
        P_VOL[0].store(0, Ordering::Relaxed);
    }
}

// Oscilloscope capture state: 0 = ready for UI read, 1 = armed (waiting trigger), 2 = capturing
static SCOPE_STATE: AtomicU8 = AtomicU8::new(0);
static mut SCOPE_BUF: [u8; SCOPE_LEN] = [0; SCOPE_LEN];
static mut SCOPE_IDX: usize = 0;
static mut SCOPE_DIV: u8 = 0;
static mut SCOPE_PREV: u8 = 0;
static mut SCOPE_WAIT: u16 = 0;

/// Logarithmic master volume steps in 1/8ths (0 = mute, 64 = full 0..2040 unclipped swing).
pub const VOL_STEPS: [u16; 10] = [0, 2, 4, 8, 12, 18, 26, 38, 50, 64];
const DEFAULT_VOL_IDX: u16 = 9; // VOL_STEPS[9] = 64 (full unclipped swing)
static VOL_IDX: AtomicU16 = AtomicU16::new(DEFAULT_VOL_IDX);
static MASTER_VOL: AtomicU16 = AtomicU16::new(VOL_STEPS[DEFAULT_VOL_IDX as usize]);

#[derive(Clone, Copy)]
pub struct VoiceInfo {
    pub active: bool,
    pub is_noise: bool,
    pub wave: u8,
    pub midi: u8,
    pub vol: u16,
}

pub struct SynthSnapshot {
    pub bar: u16,
    pub total_bars: u16,
    pub vol_idx: u16,
    pub max_vol_idx: u16,
    pub active_count: u8,
    pub paused: bool,
    pub adpcm_mode: bool,
    pub adpcm_played: u32,
    pub adpcm_total: u32,
    pub voices: [VoiceInfo; NUM_PITCH_VOICES + NUM_NOISE_VOICES],
}

pub fn snapshot() -> SynthSnapshot {
    let mut voices = [VoiceInfo {
        active: false,
        is_noise: false,
        wave: 0,
        midi: 0,
        vol: 0,
    }; NUM_PITCH_VOICES + NUM_NOISE_VOICES];
    let mut active_count = 0u8;

    for i in 0..NUM_PITCH_VOICES {
        let vol = P_VOL[i].load(Ordering::Relaxed);
        if vol > 0 {
            voices[i] = VoiceInfo {
                active: true,
                is_noise: false,
                wave: P_WAVE[i].load(Ordering::Relaxed),
                midi: P_MIDI[i].load(Ordering::Relaxed),
                vol,
            };
            active_count += 1;
        }
    }
    for i in 0..NUM_NOISE_VOICES {
        let vol = N_VOL[i].load(Ordering::Relaxed);
        if vol > 0 {
            voices[NUM_PITCH_VOICES + i] = VoiceInfo {
                active: true,
                is_noise: true,
                wave: 5,
                midi: N_MIDI[i].load(Ordering::Relaxed),
                vol,
            };
            active_count += 1;
        }
    }

    SynthSnapshot {
        bar: CUR_BAR.load(Ordering::Relaxed),
        total_bars: NUM_BARS as u16,
        vol_idx: VOL_IDX.load(Ordering::Relaxed),
        max_vol_idx: (VOL_STEPS.len() - 1) as u16,
        active_count,
        paused: PAUSED.load(Ordering::Relaxed),
        adpcm_mode: ADPCM_MODE.load(Ordering::Relaxed),
        adpcm_played: ADPCM_PLAYED_SAMPLES.load(Ordering::Relaxed),
        adpcm_total: ADPCM_TOTAL_SAMPLES.load(Ordering::Relaxed),
        voices,
    }
}

/// Copy the latest captured waveform (if ready) and re-arm the oscilloscope trigger.
pub fn read_scope(out: &mut [u8; SCOPE_LEN]) {
    if SCOPE_STATE.load(Ordering::Acquire) == 0 {
        unsafe {
            core::ptr::copy_nonoverlapping(
                (&raw const SCOPE_BUF) as *const u8,
                out.as_mut_ptr(),
                SCOPE_LEN,
            );
            SCOPE_IDX = 0;
            SCOPE_WAIT = 0;
        }
        SCOPE_STATE.store(1, Ordering::Release);
    }
}

pub fn volume_up() -> u16 {
    let idx = (VOL_IDX.load(Ordering::Relaxed) + 1).min((VOL_STEPS.len() - 1) as u16);
    VOL_IDX.store(idx, Ordering::Relaxed);
    let gain = VOL_STEPS[idx as usize];
    MASTER_VOL.store(gain, Ordering::Relaxed);
    idx
}

pub fn volume_down() -> u16 {
    let idx = VOL_IDX.load(Ordering::Relaxed).saturating_sub(1);
    VOL_IDX.store(idx, Ordering::Relaxed);
    let gain = VOL_STEPS[idx as usize];
    MASTER_VOL.store(gain, Ordering::Relaxed);
    idx
}

// Phase & LFSR state owned exclusively by the TIM3 ISR
static mut P_PHASE: [u32; NUM_PITCH_VOICES] = [0; NUM_PITCH_VOICES];
static mut N_PHASE: [u32; NUM_NOISE_VOICES] = [0; NUM_NOISE_VOICES];
static mut LFSR: [u16; NUM_NOISE_VOICES] = [0x4001, 0x2A55];

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

    // Keep TIM3 running forever (prevent Drop from disabling the timer/channel/pin).
    core::mem::forget(ch4);
    core::mem::forget(pwm);

    // Enable TIM3 update interrupt and unmask in NVIC.
    pac::TIM3.sr().modify(|w| w.set_uif(false));
    pac::TIM3.dier().modify(|w| w.set_uie(true));
    unsafe {
        pac::Interrupt::TIM3.enable();
    }
}

#[interrupt]
fn TIM3() {
    pac::TIM3.sr().modify(|w| w.set_uif(false));

    let gain = if PAUSED.load(Ordering::Relaxed) {
        0
    } else {
        MASTER_VOL.load(Ordering::Relaxed) as u32
    };

    if ADPCM_MODE.load(Ordering::Relaxed) {
        let s8 = unsafe {
            ADPCM_DIV ^= 1;
            if ADPCM_DIV == 0 {
                if ADPCM_RESET.swap(false, Ordering::Acquire) {
                    ADPCM_PRED = 0;
                    ADPCM_STEP_IDX = 0;
                    ADPCM_HAVE_HI = false;
                    ADPCM_LAST_SAMPLE = 128;
                    ADPCM_PREV_SAMPLE = 128;
                }
                if gain > 0 {
                    let nibble = if ADPCM_HAVE_HI {
                        ADPCM_HAVE_HI = false;
                        Some((ADPCM_CUR_BYTE >> 4) & 0x0F)
                    } else {
                        let tail = RING_TAIL.load(Ordering::Relaxed);
                        let head = RING_HEAD.load(Ordering::Acquire);
                        if tail != head {
                            let b = AUDIO_RING[tail & RING_MASK];
                            RING_TAIL.store(tail.wrapping_add(1), Ordering::Release);
                            ADPCM_CUR_BYTE = b;
                            ADPCM_HAVE_HI = true;
                            Some(b & 0x0F)
                        } else {
                            None
                        }
                    };

                    if let Some(nib) = nibble {
                        let step = IMA_STEP_TABLE[ADPCM_STEP_IDX as usize];
                        let mut diff = step >> 3;
                        if (nib & 4) != 0 {
                            diff += step;
                        }
                        if (nib & 2) != 0 {
                            diff += step >> 1;
                        }
                        if (nib & 1) != 0 {
                            diff += step >> 2;
                        }
                        if (nib & 8) != 0 {
                            ADPCM_PRED = (ADPCM_PRED - diff).max(-32768);
                        } else {
                            ADPCM_PRED = (ADPCM_PRED + diff).min(32767);
                        }
                        ADPCM_STEP_IDX = (ADPCM_STEP_IDX + IMA_INDEX_TABLE[nib as usize]).clamp(0, 88);
                        ADPCM_PREV_SAMPLE = ADPCM_LAST_SAMPLE;
                        ADPCM_LAST_SAMPLE = ((ADPCM_PRED + 32768) >> 8) as u8;
                        let played = ADPCM_PLAYED_SAMPLES.load(Ordering::Relaxed);
                        ADPCM_PLAYED_SAMPLES.store(played.wrapping_add(1), Ordering::Relaxed);
                    }
                }
                ADPCM_LAST_SAMPLE
            } else {
                // 2x linear interpolation on the odd 31.25 kHz tick
                ((ADPCM_PREV_SAMPLE as u16 + ADPCM_LAST_SAMPLE as u16) >> 1) as u8
            }
        };

        let duty = if gain == 0 {
            0u16
        } else {
            (((s8 as u32) * gain) >> 3).min(2047) as u16
        };
        pac::TIM3.ccr(3).write(|w| w.set_ccr(duty));

        unsafe {
            if ADPCM_DIV == 0 {
                let state = SCOPE_STATE.load(Ordering::Relaxed);
                if state != 0 {
                    let s = if gain == 0 { 128u8 } else { s8 };
                    if state == 1 {
                        SCOPE_WAIT += 1;
                        if (SCOPE_PREV < 128 && s >= 128) || SCOPE_WAIT >= 160 {
                            SCOPE_IDX = 0;
                            SCOPE_BUF[0] = s;
                            SCOPE_IDX = 1;
                            SCOPE_STATE.store(2, Ordering::Relaxed);
                        }
                    } else if state == 2 {
                        SCOPE_BUF[SCOPE_IDX] = s;
                        SCOPE_IDX += 1;
                        if SCOPE_IDX >= SCOPE_LEN {
                            SCOPE_STATE.store(0, Ordering::Release);
                        }
                    }
                    SCOPE_PREV = s;
                }
            }
        }
        return;
    }

    let mut mix: u32 = 0;

    for i in 0..NUM_PITCH_VOICES {
        let inc = P_INC[i].load(Ordering::Relaxed);
        let vol = P_VOL[i].load(Ordering::Relaxed) as u32;
        if inc == 0 || vol == 0 {
            continue;
        }
        let ph = unsafe {
            P_PHASE[i] = P_PHASE[i].wrapping_add(inc);
            P_PHASE[i]
        };
        let sample = match P_WAVE[i].load(Ordering::Relaxed) {
            // 1/8 pulse (12.5% duty)
            0 => if (ph >> 29) == 0 { vol } else { 0 },
            // 1/4 pulse (25% duty)
            1 => if (ph >> 30) == 0 { vol } else { 0 },
            // square (50% duty)
            2 => if (ph >> 31) != 0 { vol } else { 0 },
            // sawtooth (4-bit)
            3 => ((ph >> 28) * vol) >> 4,
            // double saw (two octave-detuned saws)
            _ => {
                let s1 = ph >> 28;
                let s2 = (ph.wrapping_mul(2) ^ 0x8000_0000) >> 28;
                (((s1 + s2) >> 1) * vol) >> 4
            }
        };
        mix += sample;
    }

    for i in 0..NUM_NOISE_VOICES {
        let inc = N_INC[i].load(Ordering::Relaxed);
        let vol = N_VOL[i].load(Ordering::Relaxed) as u32;
        if inc == 0 || vol == 0 {
            continue;
        }
        let bit = unsafe {
            let (new_ph, wrapped) = N_PHASE[i].overflowing_add(inc);
            N_PHASE[i] = new_ph;
            if wrapped {
                let l = LFSR[i];
                let fb = (l ^ (l >> 1)) & 1;
                LFSR[i] = (l >> 1) | (fb << 14);
            }
            LFSR[i] & 1
        };
        if bit != 0 {
            mix += vol;
        }
    }

    // Scale by master volume (in 1/8ths) and clamp to ARR = 2047
    let duty = ((mix * gain) >> 3).min(2047) as u16;
    pac::TIM3.ccr(3).write(|w| w.set_ccr(duty));

    // Capture oscilloscope sample every 2nd tick (15.625 kHz -> ~10.2 ms window)
    unsafe {
        SCOPE_DIV ^= 1;
        if SCOPE_DIV == 0 {
            let state = SCOPE_STATE.load(Ordering::Relaxed);
            if state != 0 {
                // Pre-gain visual level (0..255), flat when muted
                let s = if gain == 0 {
                    0u8
                } else {
                    (mix >> 1).min(255) as u8
                };
                if state == 1 {
                    // Wait for rising edge across mid-level, or auto-trigger after 160 samples
                    SCOPE_WAIT += 1;
                    if (SCOPE_PREV < 48 && s >= 48) || SCOPE_WAIT >= 160 {
                        SCOPE_IDX = 0;
                        SCOPE_BUF[0] = s;
                        SCOPE_IDX = 1;
                        SCOPE_STATE.store(2, Ordering::Relaxed);
                    }
                } else if state == 2 {
                    SCOPE_BUF[SCOPE_IDX] = s;
                    SCOPE_IDX += 1;
                    if SCOPE_IDX >= SCOPE_LEN {
                        SCOPE_STATE.store(0, Ordering::Release);
                    }
                }
                SCOPE_PREV = s;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn chiptune_task() {
    // Wait 0.5s after startup before playing.
    Timer::after(Duration::from_millis(500)).await;

    let mut bar: usize = 0;

    loop {
        CUR_BAR.store(bar as u16, Ordering::Relaxed);
        let bar_pats = SEQUENCE[bar];

        for ht in 0..HALF_TICKS_PER_BAR {
            while PAUSED.load(Ordering::Relaxed) {
                for v in &P_VOL {
                    v.store(0, Ordering::Relaxed);
                }
                for v in &N_VOL {
                    v.store(0, Ordering::Relaxed);
                }
                Timer::after(Duration::from_millis(20)).await;
            }

            let mut p_idx = 0usize;
            let mut n_idx = 0usize;

            for ch in 0..NUM_CHANNELS {
                let pat_id = bar_pats[ch] as usize;
                if pat_id == 0 {
                    continue;
                }
                let (start, end) = PAT_RANGES[pat_id];
                let spec = CHANNELS[ch];

                for note in &NOTES[start as usize..end as usize] {
                    if ht >= note.start_half && ht < note.end_half {
                        let age = (ht - note.start_half) as u16;

                        if spec.is_noise {
                            if n_idx < NUM_NOISE_VOICES {
                                // Crisp percussive decay for Kick/Snare/Hi-Hat
                                let env = match age {
                                    0 => 4,
                                    1 => 3,
                                    2 => 2,
                                    3 => 1,
                                    _ => 0,
                                };
                                let vol = (spec.vol as u16) * env * 2;
                                // Shift noise pitch up so LFSR clocks cleanly in audio range
                                let noise_midi = note.pitch.saturating_add(48).min(127);
                                N_MIDI[n_idx].store(note.pitch, Ordering::Relaxed);
                                N_INC[n_idx].store(midi_to_inc(noise_midi), Ordering::Relaxed);
                                N_VOL[n_idx].store(vol, Ordering::Relaxed);
                                n_idx += 1;
                            }
                        } else if p_idx < NUM_PITCH_VOICES {
                            // Gentle 1-half-tick attack + gradual sustain decay
                            let base = (spec.vol as u16) * 4;
                            let vol = if age >= 12 {
                                base / 2
                            } else {
                                base - (base * age) / 24
                            };
                            P_WAVE[p_idx].store(spec.wave, Ordering::Relaxed);
                            P_MIDI[p_idx].store(note.pitch, Ordering::Relaxed);
                            P_INC[p_idx].store(midi_to_inc(note.pitch), Ordering::Relaxed);
                            P_VOL[p_idx].store(vol, Ordering::Relaxed);
                            p_idx += 1;
                        }
                    }
                }
            }

            // Silence unused voices
            while p_idx < NUM_PITCH_VOICES {
                P_VOL[p_idx].store(0, Ordering::Relaxed);
                p_idx += 1;
            }
            while n_idx < NUM_NOISE_VOICES {
                N_VOL[n_idx].store(0, Ordering::Relaxed);
                n_idx += 1;
            }

            Timer::after(Duration::from_micros(HALF_TICK_US)).await;
        }

        bar += 1;
        if bar >= NUM_BARS {
            bar = INTRO_BARS.min(NUM_BARS.saturating_sub(1));
        }
    }
}
