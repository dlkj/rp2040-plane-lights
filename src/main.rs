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

    let mut offset = 0;

    loop {
        let (tx_buf, next_tx_transfer) = tx_transfer.wait();

        for (i, x) in [0x0f000000 >> (8 * offset)]
            .into_iter()
            .cycle()
            // .skip(offset)
            .take(tx_buf.len())
            .enumerate()
        {
            tx_buf[i] = x;
        }

        tx_buf[0] = ((tx_buf.len() as u32 - 1) * 24) - 1;

        offset = (offset + 1) % 3;

        tx_transfer = next_tx_transfer.read_next(tx_buf);

        // if offset == 0 {
        //     loop {}
        // }
    }
}
