#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal;
use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use fugit::HertzU32;
use hal::{
    clocks::init_clocks_and_plls,
    dma::DMAExt,
    gpio::{FunctionPio0, Pin, PinId},
    pac,
    pio::{Buffers::OnlyTx, PIOExt},
    sio::Sio,
    watchdog::Watchdog,
};
use panic_probe as _;
use rp_pico as bsp;
use rp_pico::hal::dma::double_buffer;
use rp_pico::hal::Clock;

type LedPin = hal::gpio::bank0::Gpio0;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let _led_pin: Pin<LedPin, FunctionPio0> = pins.gpio0.into_mode();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    const LENGTH: usize = 145;

    let mut data = [0; LENGTH];
    data[0] = ((data.len() as u32 - 1) * 24) - 1;

    #[rustfmt::skip]
    let program = pio_proc::pio_asm!(
        ".wrap_target",
        "    out x, 32",
        "next_bit:",        
        "    mov pins, null [1]",
        "    out pins, 1 [1]",
        "    mov pins, !null",
        "    jmp x-- next_bit",
        "    mov pins, null"
        "    set x, 31",
        "end_pause:",
        "    jmp x-- end_pause [8]"
        ".wrap");

    //transfer time = 1.2 microseconds
    const T1: u8 = 2; // start bit
    const T2: u8 = 2; // data bit
    const T3: u8 = 2; // stop bit
    const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;
    const FREQ: HertzU32 = HertzU32::kHz(800);

    let clock_freq = clocks.peripheral_clock.freq();

    let bit_freq = FREQ * CYCLES_PER_BIT;
    let int = clock_freq / bit_freq;
    let rem = clock_freq - (int * bit_freq);
    let frac = (rem * 256) / bit_freq;

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (mut sm, _rx, tx) = hal::pio::PIOBuilder::from_program(installed)
        .out_pins(LedPin::DYN.num, 1)
        .clock_divisor_fixed_point(int as u16, frac as u8)
        .autopull(true)
        .pull_threshold(24)
        .buffers(OnlyTx)
        .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs([(LedPin::DYN.num, hal::pio::PinDir::Output)]);
    sm.start();

    delay.delay_ms(10);

    let dma = pac.DMA.split(&mut pac.RESETS);
    let ch0 = dma.ch0;
    let ch1 = dma.ch1;

    let tx_buf1 = singleton!(TX_BUFFER_1: [u32; LENGTH] = data).unwrap();
    let tx_buf2 = singleton!(TX_BUFFER_2: [u32; LENGTH] = data).unwrap();

    let tx_transfer = double_buffer::Config::new((ch0, ch1), tx_buf1, tx).start();
    let mut tx_transfer = tx_transfer.read_next(tx_buf2);

    let mut t = 0.0;
    let sin = hal::rom_data::float_funcs::fsin::ptr();

    loop {
        let (tx_buf, next_tx_transfer) = tx_transfer.wait();

        for (i, led) in tx_buf.iter_mut().skip(1).enumerate() {
            let hue_offs = (i as f32) / (2 * LENGTH) as f32;

            let sin_11 = sin((t + hue_offs) * 2.0 * core::f32::consts::PI);
            // Bring -1..1 sine range to 0..1 range:
            let sin_01 = (sin_11 + 1.0) * 0.5;

            let hue = 360.0 * sin_01;
            let sat = 1.0;
            let val = 1.0;

            let (r, g, b) = brightness(64, gamma(hsv2rgb_u8(hue, sat, val)));
            let rgb = u32::from(g) << 24 | u32::from(r) << 16 | u32::from(b) << 8;
            *led = rgb;
        }

        tx_buf[0] = ((tx_buf.len() as u32 - 1) * 24) - 1;

        t += 1.0 / 2000.0;
        while t > 1.0 {
            t -= 1.0;
        }

        tx_transfer = next_tx_transfer.read_next(tx_buf);
    }
}

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r, g, b) = if hue < 60.0 {
        (c, x, 0.0)
    } else if hue < 120.0 {
        (x, c, 0.0)
    } else if hue < 180.0 {
        (0.0, c, x)
    } else if hue < 240.0 {
        (0.0, x, c)
    } else if hue < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    (r + m, g + m, b + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);

    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8,
    )
}

fn brightness(brightness: u8, (r, g, b): (u8, u8, u8)) -> (u8, u8, u8) {
    (
        (r as u16 * (brightness as u16 + 1) / 256) as u8,
        (g as u16 * (brightness as u16 + 1) / 256) as u8,
        (b as u16 * (brightness as u16 + 1) / 256) as u8,
    )
}

fn gamma((r, g, b): (u8, u8, u8)) -> (u8, u8, u8) {
    // This table remaps linear input values
    // (the numbers we’d like to use; e.g. 127 = half brightness)
    // to nonlinear gamma-corrected output values
    // (numbers producing the desired effect on the LED;
    // e.g. 36 = half brightness).
    const GAMMA8: [u8; 256] = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
        4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12,
        13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24,
        24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40,
        41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
        64, 66, 67, 68, 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89, 90, 92, 93,
        95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114, 115, 117, 119, 120, 122, 124,
        126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158,
        160, 162, 164, 167, 169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 196, 198,
        200, 203, 205, 208, 210, 213, 215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244,
        247, 249, 252, 255,
    ];
    (GAMMA8[r as usize], GAMMA8[g as usize], GAMMA8[b as usize])
}
