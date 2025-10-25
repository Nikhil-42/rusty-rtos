#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::fmt::Write;
use eel4745c::rtos::{self, G8torMutex, G8torMutexHandle, G8torRtosHandle};
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use tm4c123x_hal::{
    self as hal,
    gpio::{
        gpioa::{PA0, PA1},
        gpiof::{PF1, PF2},
        AlternateFunction, Output, PushPull, AF1,
    },
    pac::{self, UART0},
    prelude::*,
    serial::Serial,
};

use cortex_m_rt::entry;

static mut R_LED_S: Option<PF1<Output<PushPull>>> = None;
static mut B_LED_S: Option<PF2<Output<PushPull>>> = None;
static mut UART0_HANDLE: Option<
    G8torMutexHandle<
        Serial<
            UART0,
            PA1<AlternateFunction<AF1, PushPull>>,
            PA0<AlternateFunction<AF1, PushPull>>,
            (),
            (),
        >,
    >,
> = None;
static UART0_MUTEX: G8torMutex<
    Serial<
        UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
> = G8torMutex::empty();

extern "C" fn blink_red(rtos: G8torRtosHandle) -> ! {
    let mut r_led = unsafe { R_LED_S.take() }.expect("Red LED is initialized.");
    let uart_handle = unsafe { UART0_HANDLE.as_ref().expect("UART0 handle is initialized.") };

    loop {
        r_led
            .set_state(embedded_hal::digital::PinState::Low)
            .unwrap();
        rtos.sleep_ms(50);
        r_led
            .set_state(embedded_hal::digital::PinState::High)
            .unwrap();
        rtos.sleep_ms(50);

        let uart = UART0_MUTEX.get(rtos.take_mutex(uart_handle));
        writeln!(uart, "Red LED blinked!\r").unwrap();
        rtos.release_mutex(uart_handle, UART0_MUTEX.release(uart));
    }
}

extern "C" fn blink_blue(rtos: G8torRtosHandle) -> ! {
    let mut b_led = unsafe { B_LED_S.take() }.expect("Blue LED is initialized.");
    let uart_handle = unsafe { UART0_HANDLE.as_ref().expect("UART0 handle is initialized.") };

    loop {
        b_led
            .set_state(embedded_hal::digital::PinState::Low)
            .unwrap();
        // rtos.sleep_ms(50);
        cortex_m::asm::delay(800_000); // approx 50ms at 16MHz
        b_led
            .set_state(embedded_hal::digital::PinState::High)
            .unwrap();
        cortex_m::asm::delay(800_000); // approx 50ms at 80MHz
                                       // rtos.sleep_ms(50);

        let uart = UART0_MUTEX.get(rtos.take_mutex(uart_handle));
        writeln!(uart, "Blue LED blinked!\r").unwrap();
        rtos.release_mutex(uart_handle, UART0_MUTEX.release(uart));
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

    let portf = p.GPIO_PORTF.split(&sc.power_control);
    let mut porta = p.GPIO_PORTA.split(&sc.power_control);

    // Activate UART
    let uart = hal::serial::Serial::uart0(
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
        let _ = R_LED_S.insert(portf.pf1.into_push_pull_output());
        let _ = B_LED_S.insert(portf.pf2.into_push_pull_output());
        UART0_MUTEX.init(uart);
    }

    unsafe {
        let inst = rtos::G8torRtos::new(pac::CorePeripherals::take().unwrap());
        UART0_HANDLE = Some(
            inst.init_mutex(&UART0_MUTEX)
                .expect("We haven't run out of atomics"),
        );
        let _ = inst
            .add_thread(b"blk_red\0\0\0\0\0\0\0\0\0", blink_red)
            .expect("Failed to add red thread");
        let _ = inst
            .add_thread(b"blk_blu\0\0\0\0\0\0\0\0\0", blink_blue)
            .expect("Failed to add blue thread");
        inst.launch()
    }
}
