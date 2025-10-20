#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use eel4745c::rtos;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use tm4c123x_hal::{self as hal, pac, gpio::{gpiof::{PF1, PF2}, Output, PushPull}, prelude::*};

use cortex_m_rt::entry;

static mut R_LED_S: Option<PF1<Output<PushPull>>> = None;
static mut B_LED_S: Option<PF2<Output<PushPull>>> = None;

extern "C" fn blink_red() -> ! {
    let mut r_led = unsafe { R_LED_S.take() }.expect("Red LED is initialized.");
    let mut state = false;

    loop {
        r_led.set_state(state.into()).unwrap();
        state = !state;
        cortex_m::asm::delay(500_000_000 / 12);
    }
}

extern "C" fn blink_blue() -> ! {
    let mut b_led = unsafe { B_LED_S.take() }.expect("Blue LED is initialized.");
    let mut state = false;

    loop {
        b_led.set_state(state.into()).unwrap();
        state = !state;
        cortex_m::asm::delay(1000_000_000 / 12);
    }
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );

    let portf = p.GPIO_PORTF.split(&sc.power_control);

    unsafe {
        let _ = R_LED_S.insert(portf.pf1.into_push_pull_output());
        let _ = B_LED_S.insert(portf.pf2.into_push_pull_output());
    }

    unsafe {
        let inst = rtos::G8torRtos::new(pac::CorePeripherals::take().unwrap());
        let _ = inst.add_thread(b"blk_red\0\0\0\0\0\0\0\0\0", blink_red).expect("Failed to add red thread");
        let _ = inst.add_thread(b"blk_blu\0\0\0\0\0\0\0\0\0", blink_blue).expect("Failed to add blue thread");
        inst.launch()
    }
}
