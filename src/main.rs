#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use rp_pico as bsp;

    use bsp::{hal, XOSC_CRYSTAL_FREQ};
    use defmt::info;
    use embedded_hal::PwmPin;
    use hal::{clocks::init_clocks_and_plls, sio::Sio, watchdog::Watchdog};
    use rp2040_monotonic::Rp2040Monotonic;
    use rp_pico::hal::gpio::bank0::Gpio1;
    use rp_pico::hal::gpio::Interrupt::EdgeLow;
    use rp_pico::hal::gpio::{FunctionPwm, Pin, PullUpDisabled};
    use rp_pico::hal::pwm::{InputHighRunning, Pwm0, Slice};

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type AppMonotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        pwm: Slice<Pwm0, InputHighRunning>,
        pin: Pin<Gpio1, FunctionPwm>,
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
        init_clocks_and_plls(
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

        let mono = Rp2040Monotonic::new(cx.device.TIMER);

        unsafe {
            hal::pac::NVIC::unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
        }

        (Shared {}, Local { pwm, pin }, init::Monotonics(mono))
    }

    #[task(binds = IO_IRQ_BANK0, local = [pwm, pin])]
    fn io_irq_bank0(cx: io_irq_bank0::Context) {
        let width = cx.local.pwm.get_counter();
        cx.local.pwm.set_counter(0);
        if width > 50 {
            info!("pulse: {}us", width);
        }
        cx.local.pin.clear_interrupt(EdgeLow);
        //expect 1000-2000 us, centred around 1500 us at 50hz
    }
}
