#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use hal::{prelude::_rphal_pio_PIOExt, Timer};
use panic_probe as _;
use rp_pico as bsp;
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Instanciate a Ws2812 LED strip:
    let mut ws = Ws2812::new(
        pins.gpio0.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    const PORT_FORWARD: usize = 12;
    const PORT_REAR: usize = 11;
    const STARBOARD_FORWARD: usize = 4;
    const STARBOARD_REAR: usize = 5;

    const TAIL: usize = 8;
    const BEACON: usize = 6;

    let mut leds: [RGB8; 16] = [
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (0, 0, 0).into(),
    ];

    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        led_pin.set_high().unwrap();

        leds[PORT_FORWARD] = (255, 0, 0).into();
        leds[PORT_REAR] = (64, 64, 64).into();
        leds[STARBOARD_FORWARD] = (0, 255, 0).into();
        leds[STARBOARD_REAR] = (64, 64, 64).into();
        leds[TAIL] = (64, 64, 64).into();
        ws.write(leds.iter().copied()).unwrap();

        delay.delay_ms(300);

        leds[BEACON] = (255, 0, 0).into();
        ws.write(leds.iter().copied()).unwrap();

        delay.delay_ms(300);

        leds[BEACON] = (0, 0, 0).into();
        ws.write(leds.iter().copied()).unwrap();

        delay.delay_ms(300);

        led_pin.set_low().unwrap();

        leds[PORT_FORWARD] = (255, 255, 255).into();
        leds[PORT_REAR] = (255, 255, 255).into();
        leds[STARBOARD_FORWARD] = (255, 255, 255).into();
        leds[STARBOARD_REAR] = (255, 255, 255).into();
        leds[TAIL] = (255, 255, 255).into();
        ws.write(leds.iter().copied()).unwrap();

        delay.delay_ms(33);

        leds[PORT_FORWARD] = (255, 0, 0).into();
        leds[PORT_REAR] = (64, 64, 64).into();
        leds[STARBOARD_FORWARD] = (0, 255, 0).into();
        leds[STARBOARD_REAR] = (64, 64, 64).into();
        leds[TAIL] = (64, 64, 64).into();
        ws.write(leds.iter().copied()).unwrap();

        delay.delay_ms(33);

        leds[PORT_FORWARD] = (255, 255, 255).into();
        leds[PORT_REAR] = (255, 255, 255).into();
        leds[STARBOARD_FORWARD] = (255, 255, 255).into();
        leds[STARBOARD_REAR] = (255, 255, 255).into();
        leds[TAIL] = (255, 255, 255).into();
        ws.write(leds.iter().copied()).unwrap();

        delay.delay_ms(33);
    }
}
