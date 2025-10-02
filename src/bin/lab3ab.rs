#![no_std]
#![no_main]

use eel4745c::rtos;
use panic_semihosting as _;

use core::fmt::Write;
use tm4c123x_hal::{self as hal, pac, prelude::*};

use cortex_m_rt::entry;

fn blink_red() -> ! {
    loop {
        // Delay
        // for _ in 0..5_000_000 {
            cortex_m::asm::nop();
        // }
    }
}

fn blink_blue() -> !{
    loop {
        // Delay
        // for _ in 0..5_000_000 {
            cortex_m::asm::nop();
        // }
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
    let mut uart = hal::serial::Serial::uart0(
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

    unsafe {
        let inst = rtos::G8torRtos::new(pac::CorePeripherals::take().unwrap());
        let red_thread = inst.add_thread(blink_red);
        let blue_thread = inst.add_thread(blink_blue);
        inst.launch();
    }
}
