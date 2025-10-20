#![no_std]
#![no_main]

use cortex_m::delay;
use eel4745c::rtos;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use tm4c123x_hal::{self as hal, gpio::{gpioa::{PA0, PA1}, gpiof::{PF1, PF2, PF3}, AlternateFunction, Output, PushPull, AF1}, pac::{self, UART0}, prelude::*, serial::Serial};

use cortex_m_rt::entry;

static mut UART0_S: Option<Serial<UART0, PA1<AlternateFunction<AF1, PushPull>>, PA0<AlternateFunction<AF1, PushPull>>, (), ()>> = None;
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
    let clocks = sc.clock_setup.freeze();

    let mut porta = p.GPIO_PORTA.split(&sc.power_control);

    // Activate UART
    let mut uart: tm4c123x_hal::serial::Serial<pac::UART0, tm4c123x_hal::gpio::gpioa::PA1<tm4c123x_hal::gpio::AlternateFunction<tm4c123x_hal::gpio::AF1, PushPull>>, tm4c123x_hal::gpio::gpioa::PA0<tm4c123x_hal::gpio::AlternateFunction<tm4c123x_hal::gpio::AF1, PushPull>>, (), ()> = hal::serial::Serial::uart0(
        p.UART0,
        porta
            .pa1
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        porta
            .pa0
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        (),
        (),
        115200_u32.bps(),
        hal::serial::NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control,
    );

    let portf = p.GPIO_PORTF.split(&sc.power_control);

    unsafe {
        R_LED_S.insert(portf.pf1.into_push_pull_output());
        B_LED_S.insert(portf.pf2.into_push_pull_output());
        UART0_S.insert(uart);
    }



    unsafe {
        let inst = rtos::G8torRtos::get(pac::CorePeripherals::take().unwrap());
        let red_thread = inst.add_thread(blink_red).expect("Failed to add red thread");
        let blue_thread = inst.add_thread(blink_blue).expect("Failed to add blue thread");
        inst.launch()
    }
}
