#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    const WING_LEN: usize = 17;
    const BODY_LEN: usize = 15;

    use bsp::{hal, XOSC_CRYSTAL_FREQ};
    use core::iter::{once, zip};
    use embedded_hal::PwmPin;
    use hal::rom_data::float_funcs::fsin;
    use hal::{
        clocks::init_clocks_and_plls,
        gpio::{
            bank0::Gpio16, bank0::Gpio17, bank0::Gpio18, bank0::Gpio27, FunctionPwm,
            Interrupt::EdgeLow, Pin, PullUpDisabled,
        },
        pac,
        pio::{self, PIOExt},
        pwm::{InputHighRunning, Pwm5, Slice},
        sio::Sio,
        watchdog::Watchdog,
        Clock,
    };
    use itertools::{repeat_n, unfold};
    use rp2040_monotonic::fugit::{ExtU64, TimerInstantU64};
    use rp2040_monotonic::Rp2040Monotonic;
    use rp_pico as bsp;
    use smart_leds::SmartLedsWrite;
    use ws2812_pio::Ws2812Direct;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type AppMonotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {
        pwm_value: u16,
    }

    #[local]
    struct Local {
        pwm: Slice<Pwm5, InputHighRunning>,
        pin: Pin<Gpio27, FunctionPwm>,
        ws_body: Ws2812Direct<pac::PIO0, pio::SM0, Gpio16>,
        ws_left: Ws2812Direct<pac::PIO0, pio::SM1, Gpio17>,
        ws_right: Ws2812Direct<pac::PIO0, pio::SM2, Gpio18>,
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

        pins.gpio27.set_interrupt_enabled(EdgeLow, true);
        let pwm_slices = hal::pwm::Slices::new(cx.device.PWM, &mut resets);

        let mut pwm: Slice<_, InputHighRunning> = pwm_slices.pwm5.into_mode();
        pwm.set_div_int(125); // 1us pwm slice clock
        pwm.enable();

        let channel = &mut pwm.channel_b;
        let pin: Pin<_, PullUpDisabled> = channel.input_from(pins.gpio27).into_mode();
        let pin: Pin<_, FunctionPwm> = pin.into_mode();
        channel.enable();

        let (mut pio0, sm0, sm1, sm2, _) = cx.device.PIO0.split(&mut resets);

        let ws_body = Ws2812Direct::new(
            pins.gpio16.into_mode(),
            &mut pio0,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        let ws_left = Ws2812Direct::new(
            pins.gpio17.into_mode(),
            &mut pio0,
            sm1,
            clocks.peripheral_clock.freq(),
        );

        let ws_right = Ws2812Direct::new(
            pins.gpio18.into_mode(),
            &mut pio0,
            sm2,
            clocks.peripheral_clock.freq(),
        );

        let mono = Rp2040Monotonic::new(cx.device.TIMER);

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        }

        write_led::spawn().unwrap();

        (
            Shared { pwm_value: 0 },
            Local {
                pwm,
                pin,
                ws_body,
                ws_left,
                ws_right,
            },
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
        local = [ws_body, ws_left, ws_right]
    )]
    fn write_led(mut cx: write_led::Context) {
        let pwm_value: u16 = cx.shared.pwm_value.lock(|p| *p);
        let now: TimerInstantU64<1_000_000> = monotonics::now();
        let now = (now.ticks() & u64::from(u32::MAX)) as f32 / 1_000_000.0;

        let start = (now / 20.0) % 1.0;
        const STEP: f32 = 0.1 / BODY_LEN as f32;

        let spectrum_iter_right = unfold(start, |t| {
            let colour = spectrum(*t);
            *t += STEP;
            while *t > 1.0 {
                *t -= 1.0;
            }
            Some::<(u8, u8, u8)>(colour)
        });

        let spectrum_iter_left = unfold(start, |t| {
            let colour = spectrum(*t);
            *t -= STEP;
            while *t < 0.0 {
                *t += 1.0;
            }
            Some::<(u8, u8, u8)>(colour)
        });

        if (1700..2200).contains(&pwm_value) {
            cx.local
                .ws_body
                .write(body_nav(now).map(|x| x.unwrap_or_default()))
                .unwrap();

            cx.local
                .ws_left
                .write(wing_nav(now, WingSide::Left).map(|x| x.unwrap_or_default()))
                .unwrap();

            cx.local
                .ws_right
                .write(wing_nav(now, WingSide::Right).map(|x| x.unwrap_or_default()))
                .unwrap();
        } else {
            cx.local
                .ws_body
                .write(zip(body_nav(now), spectrum_iter_left.clone()).map(|(a, b)| a.apply(b)))
                .unwrap();

            cx.local
                .ws_left
                .write(
                    zip(wing_nav(now, WingSide::Left), spectrum_iter_left.clone())
                        .map(|(a, b)| a.apply(b)),
                )
                .unwrap();

            cx.local
                .ws_right
                .write(
                    zip(wing_nav(now, WingSide::Right), spectrum_iter_right)
                        .map(|(a, b)| a.apply(b)),
                )
                .unwrap();
        }

        let next = 1.millis();
        write_led::spawn_after(next).unwrap();
    }

    fn wing_nav(now: f32, side: WingSide) -> impl Iterator<Item = Overlay> {
        let sec_frac = now % 1.0;

        let nav_colour = if (0.0..0.033).contains(&sec_frac) || (0.066..0.099).contains(&sec_frac) {
            (255, 255, 255)
        } else {
            match side {
                WingSide::Left => (64, 0, 0),
                WingSide::Right => (0, 64, 0),
            }
        };

        repeat_n(Overlay::Transparent, WING_LEN - 6)
            .chain(repeat_n(Overlay::Brightness(16), 4))
            .chain(repeat_n(Overlay::Colour(nav_colour), 2))
    }

    fn body_nav(now: f32) -> impl Iterator<Item = Overlay> {
        let sec_frac = now % 1.0;

        let beacon_color = if (0.35..0.65).contains(&sec_frac) {
            Overlay::Colour((255, 0, 0))
        } else {
            Overlay::Transparent
        };

        let nav_colour = if (0.0..0.033).contains(&sec_frac) || (0.066..0.099).contains(&sec_frac) {
            (255, 255, 255)
        } else {
            (64, 64, 64)
        };

        repeat_n(Overlay::Transparent, 2)
            .chain(once(beacon_color))
            .chain(repeat_n(Overlay::Transparent, BODY_LEN - 4))
            .chain(once(Overlay::Colour(nav_colour)))
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

    enum WingSide {
        Left,
        Right,
    }

    #[derive(Clone)]
    enum Overlay {
        Transparent,
        Colour((u8, u8, u8)),
        Brightness(u8),
    }

    impl Overlay {
        pub fn unwrap_or_default(self) -> (u8, u8, u8) {
            match self {
                Self::Colour(c) => c,
                _ => (0, 0, 0),
            }
        }

        pub fn apply(self, c: (u8, u8, u8)) -> (u8, u8, u8) {
            match self {
                Self::Colour(c) => c,
                Self::Transparent => c,
                Self::Brightness(b) => brightness(c, b),
            }
        }
    }

    fn brightness((r, g, b): (u8, u8, u8), brightness: u8) -> (u8, u8, u8) {
        (
            (r as u16 * (brightness as u16 + 1) / 256) as u8,
            (g as u16 * (brightness as u16 + 1) / 256) as u8,
            (b as u16 * (brightness as u16 + 1) / 256) as u8,
        )
    }
}
