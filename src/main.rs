#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    const LED_COUNT: usize = 144;

    use bsp::{hal, XOSC_CRYSTAL_FREQ};
    use core::iter::{repeat, zip};
    use embedded_hal::PwmPin;
    use hal::rom_data::float_funcs::fsin;
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
    use itertools::unfold;
    use rp2040_monotonic::fugit::{ExtU64, TimerInstantU64};
    use rp2040_monotonic::Rp2040Monotonic;
    use rp_pico as bsp;
    use smart_leds::{brightness, SmartLedsWrite};
    use ws2812_pio::Ws2812Direct;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type AppMonotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {
        pwm_value: u16,
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
            Shared { pwm_value: 0 },
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
        cx.shared.pwm_value.lock(|p| *p = width);
        cx.local.pin.clear_interrupt(EdgeLow);
    }

    #[task(
        shared = [pwm_value],
        local = [ws]
    )]
    fn write_led(mut cx: write_led::Context) {
        let pwm_value: u16 = cx.shared.pwm_value.lock(|p| *p);
        let now: TimerInstantU64<1_000_000> = monotonics::now();
        let now = (now.ticks() & u64::from(u32::MAX)) as f32 / 1_000_000.0;

        let start = (now / 10.0) % 1.0;
        const STEP: f32 = 0.5 / LED_COUNT as f32;

        let nav = repeat(Some((64, 0, 0).into()))
            .take(4)
            .chain(repeat(None).take(LED_COUNT / 2 - 6))
            .chain(
                repeat(if now % 1.0 < 0.333 {
                    Some((255, 0, 0).into())
                } else {
                    None
                })
                .take(4),
            )
            .chain(repeat(None).take(LED_COUNT / 2 - 6))
            .chain(repeat(Some((0, 64, 0).into())));

        let spectrum_iter = unfold(start, |t| {
            let colour = spectrum(*t);
            *t += STEP;
            while *t > 1.0 {
                *t -= 1.0;
            }
            Some(colour.into())
        });

        if pwm_value > 1700 && pwm_value < 2200 {
            let mux = zip(nav, brightness(spectrum_iter, 32)).map(|(a, b)| a.unwrap_or(b));
            cx.local.ws.write(mux.take(LED_COUNT)).unwrap();
        } else {
            cx.local
                .ws
                .write(nav.map(|x| x.unwrap_or_default()).take(LED_COUNT))
                .unwrap();
        }

        let next = 60.millis();
        write_led::spawn_after(next).unwrap();
    }

    fn spectrum(offset: f32) -> (u8, u8, u8) {
        let sin_11 = fsin(offset * 2.0 * core::f32::consts::PI);
        // Bring -1..1 sine range to 0..1 range:
        let sin_01 = (sin_11 + 1.0) * 0.5;

        hsv2rgb_u8(360.0 * sin_01, 1.0, 1.0)
    }

    fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
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

    fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
        let r = hsv2rgb(h, s, v);

        (
            (r.0 * 255.0) as u8,
            (r.1 * 255.0) as u8,
            (r.2 * 255.0) as u8,
        )
    }
}
