#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use tm4c123x_hal::{self as hal, pac, delay, prelude::*};

use embedded_hal::{delay::DelayNs, digital::{InputPin, OutputPin}};

enum State {
    Red,
    Green,
    Blue,
}

impl State {
    fn next(&self) -> State {
        match self {
            State::Red => State::Green,
            State::Green => State::Blue,
            State::Blue => State::Red,
        }
    }
}

#[entry]
fn main() -> ! {
    let cortex_p = cortex_m::Peripherals::take().unwrap();
    let p = pac::Peripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    let portf = p.GPIO_PORTF.split(&sc.power_control);
    let mut delay_provider = delay::Delay::new(cortex_p.SYST, &clocks);

    let mut r_led_pin = portf.pf1.into_push_pull_output();
    let mut g_led_pin = portf.pf3.into_push_pull_output();
    let mut b_led_pin = portf.pf2.into_push_pull_output();

    let mut active_led = State::Red;
    let mut sw1_pin = portf.pf4.into_pull_up_input();

    loop {
        match active_led {
            State::Red => {
                r_led_pin.set_high().unwrap();
                g_led_pin.set_low().unwrap();
                b_led_pin.set_low().unwrap();
            }
            State::Green => {
                r_led_pin.set_low().unwrap();
                g_led_pin.set_high().unwrap();
                b_led_pin.set_low().unwrap();
            }
            State::Blue => {
                r_led_pin.set_low().unwrap();
                g_led_pin.set_low().unwrap();
                b_led_pin.set_high().unwrap();
            }
        }

        // Debounce
        while sw1_pin.is_high().unwrap() {}
        delay_provider.delay_ms(5u32);
        if sw1_pin.is_high().unwrap() {
            continue;
        }
        
        // Debounce
        while sw1_pin.is_low().unwrap() {}
        delay_provider.delay_ms(5u32);
        if sw1_pin.is_low().unwrap() {
            continue;
        }

        active_led = active_led.next();
    }
}
