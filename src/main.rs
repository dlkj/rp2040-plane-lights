#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use bsp::{hal, XOSC_CRYSTAL_FREQ};
    use embedded_hal::PwmPin;
    use hal::{
        clocks::init_clocks_and_plls,
        gpio::{bank0::Gpio0, bank0::Gpio1, FunctionPwm, Interrupt::EdgeLow, Pin, PullUpDisabled},
        pac,
        pio::{self, PIOExt},
        pwm::{InputHighRunning, Pwm0, Slice},
        sio::Sio,
        watchdog::Watchdog,
        Clock,
    };
    use rp2040_monotonic::fugit::{ExtU64, TimerInstantU64};
    use rp2040_monotonic::Rp2040Monotonic;
    use rp_pico as bsp;
    use smart_leds::{SmartLedsWrite, RGB8};
    use ws2812_pio::Ws2812Direct;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type AppMonotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {
        pwm_value: Option<u16>,
    }

    #[local]
    struct Local {
        pwm: Slice<Pwm0, InputHighRunning>,
        pin: Pin<Gpio1, FunctionPwm>,
        ws: Ws2812Direct<pac::PIO0, pio::SM0, Gpio0>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        let mut resets = cx.device.RESETS;
        let mut watchdog = Watchdog::new(cx.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(cx.device.SIO);
        let pins = rp_pico::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        pins.gpio1.set_interrupt_enabled(EdgeLow, true);
        let pwm_slices = hal::pwm::Slices::new(cx.device.PWM, &mut resets);

        let mut pwm: Slice<_, InputHighRunning> = pwm_slices.pwm0.into_mode();
        pwm.set_div_int(125); // 1us pwm slice clock
        pwm.enable();

        let channel = &mut pwm.channel_b;
        let pin: Pin<_, PullUpDisabled> = channel.input_from(pins.gpio1).into_mode();
        let pin: Pin<_, FunctionPwm> = pin.into_mode();
        channel.enable();

        let (mut pio, sm0, _, _, _) = cx.device.PIO0.split(&mut resets);

        let ws = Ws2812Direct::new(
            pins.gpio0.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        let mono = Rp2040Monotonic::new(cx.device.TIMER);

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        }

        write_led::spawn().unwrap();

        (
            Shared { pwm_value: None },
            Local { pwm, pin, ws },
            init::Monotonics(mono),
        )
    }

    #[task(
        binds = IO_IRQ_BANK0,
        shared = [pwm_value],
        local = [pwm, pin]
    )]
    fn io_irq_bank0(mut cx: io_irq_bank0::Context) {
        let width = cx.local.pwm.get_counter();
        cx.local.pwm.set_counter(0);
        cx.shared.pwm_value.lock(|p| *p.insert(width));
        cx.local.pin.clear_interrupt(EdgeLow);
    }

    #[task(
        shared = [pwm_value],
        local = [ws]
    )]
    fn write_led(mut cx: write_led::Context) {
        let pwm_value: Option<u16> = cx.shared.pwm_value.lock(|p| p.clone());
        let now: TimerInstantU64<1_000_000> = monotonics::now();

        let color: RGB8 = (255, 0, 255).into();
        cx.local.ws.write([color].into_iter()).unwrap();

        let next = 60.millis();
        write_led::spawn_after(next).unwrap();
    }
}
