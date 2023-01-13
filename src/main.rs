#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal;
use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use hal::{
    clocks::init_clocks_and_plls,
    dma::{double_buffer, DMAExt},
    pac,
    pio::PIOExt,
    sio::Sio,
    watchdog::Watchdog,
};
use panic_probe as _;
use rp_pico as bsp;
use rp_pico::hal::gpio::{FunctionPio0, Pin};
use rp_pico::hal::pio::Buffers::OnlyTx;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let _clocks = init_clocks_and_plls(
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

    let _led_pin: Pin<_, FunctionPio0> = pins.led.into_mode();
    const LED_PIN_ID: u8 = 25;

    // HELLO WORLD in morse code:
    // .... . .-.. .-.. --- / .-- --- .-. .-.. -..
    #[allow(clippy::unusual_byte_groupings)]
    let message = [
        0b10101010_00100010_11101010_00101110,
        0b10100011_10111011_10000000_10111011,
        0b10001110_11101110_00101110_10001011,
        0b10101000_11101010_00000000_00000000,
        0b00000000_00000000_00000000_00000000,
    ];

    #[rustfmt::skip]
    let program = pio_proc::pio_asm!(
        ".wrap_target",
        "    out pins, 1 [31]",
        "    nop [31]",
        ".wrap");

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (mut sm, _rx, tx) = hal::pio::PIOBuilder::from_program(installed)
        .out_pins(LED_PIN_ID, 1)
        .clock_divisor_fixed_point(0, 0) // as slow as possible (0 is interpreted as 65536)
        .autopull(true)
        .buffers(OnlyTx)
        .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs([(LED_PIN_ID, hal::pio::PinDir::Output)]);
    sm.start();

    let dma = pac.DMA.split(&mut pac.RESETS);

    let tx_buf1 = singleton!(TX_BUFFER_1: [u32; 5] = message).unwrap();
    let tx_buf2 = singleton!(TX_BUFFER_2: [u32; 5] = message).unwrap();

    let tx_transfer = double_buffer::Config::new((dma.ch0, dma.ch1), tx_buf1, tx).start();
    let mut tx_transfer = tx_transfer.read_next(tx_buf2);

    loop {
        // When a transfer is done we immediately enqueue the buffers again.
        if tx_transfer.is_done() {
            info!("Buffer done");
            let (tx_buf, next_tx_transfer) = tx_transfer.wait();
            tx_transfer = next_tx_transfer.read_next(tx_buf);
        }
    }
}
