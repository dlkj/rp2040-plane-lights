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

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type AppMonotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

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

        let mut pwm_slices = hal::pwm::Slices::new(cx.device.PWM, &mut resets);

        let led_pwm = &mut pwm_slices.pwm4;
        led_pwm.set_div_int(0);
        led_pwm.enable();

        let led_channel = &mut led_pwm.channel_b;
        led_channel.output_to(pins.led);
        led_channel.set_duty(led_channel.get_max_duty() / 16);

        led_pwm.enable_interrupt();

        let mono = Rp2040Monotonic::new(cx.device.TIMER);

        unsafe {
            hal::pac::NVIC::unmask(hal::pac::Interrupt::PWM_IRQ_WRAP);
        }

        (Shared {}, Local {}, init::Monotonics(mono))
    }

    #[task(binds = PWM_IRQ_WRAP)]
    fn pwm_wrap_irq(_cx: pwm_wrap_irq::Context) {
        info!("PWM WRAP");
    }
}
