#![no_std]
#![no_main]

use panic_semihosting as _;

use core::fmt::Write;
use cortex_m_rt::entry;
use tm4c123x_hal::{self as hal, pac::{self, GPIO_PORTE, NVIC}, prelude::*};
use pac::interrupt;

#[interrupt]
fn GPIOD() {
    // Interrupt handler for GPIO Port D
    // (Implementation would go here)
    let portd = unsafe { &*pac::GPIO_PORTD::ptr() };
    portd.icr.write(|w| unsafe { w.gpio().bits(1 << 2) }); // Clear pin 2
}

#[interrupt]
fn GPIOE() {
    // Interrupt handler for GPIO Port E
    // (Implementation would go here)
    let porte = unsafe { &*pac::GPIO_PORTE::ptr() };
    porte.icr.write(|w| unsafe { w.gpio().bits(1 << 4) }); // Clear pin 2
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let mut core_p = pac::CorePeripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let portd = p.GPIO_PORTD.split(&sc.power_control);
    let porte = p.GPIO_PORTE.split(&sc.power_control);

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

    // Enable buttons interrupt
    let mut pe4 = porte.pe4.into_pull_up_input();
    pe4.set_interrupt_mode(hal::gpio::InterruptMode::EdgeFalling);

    // Initialize Joystick ADCs
    // let pe2, pe3

    // Enable Joystick Interrupt
    let mut pd2 = portd.pd2.into_pull_up_input();
    pd2.set_interrupt_mode(hal::gpio::InterruptMode::EdgeFalling);

    // Enable the interrupt in the NVIC
    let nvic = &mut core_p.NVIC;
    unsafe {
        nvic.set_priority(pac::interrupt::GPIOD, 7 << 5);
        NVIC::unmask(pac::interrupt::GPIOD);
        nvic.set_priority(pac::interrupt::GPIOE, 6 << 5);
        NVIC::unmask(pac::interrupt::GPIOE);
    }

    loop {
        writeln!(uart, "I live!!! \n-Rust").unwrap();
    }
}
