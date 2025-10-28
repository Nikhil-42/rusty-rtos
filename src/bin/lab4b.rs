#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use eel4745c::rtos::{self, G8torFifoHandle, G8torMutex, G8torMutexHandle, G8torRtosHandle};
use eh0::{serial::{Read, Write}};
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
static mut UART0_TX_FIFO: Option<
    G8torFifoHandle
> = None;
static mut UART0_RX_FIFO: Option<
    G8torFifoHandle
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

extern "C" fn publisher(rtos: G8torRtosHandle) -> ! {
    let tx_fifo_handle = unsafe { UART0_TX_FIFO.as_ref().expect("UART0 TX FIFO handle is initialized.") };
    let mut count: u8 = 0;

    loop {
        rtos.write_fifo(tx_fifo_handle, count as u32);
        rtos.sleep_ms(100);
        count = count.wrapping_add(1);
    }
}

// extern "C" fn consumer(rtos: G8torRtosHandle) -> ! {
//     let rx_fifo_handle = unsafe { UART0_RX_FIFO.as_ref().expect("UART0 RX FIFO handle is initialized.") };

//     loop {
//         let val = rtos.read_fifo(rx_fifo_handle);
//         rtos.sleep_ms(val as usize);
//     }
// }

extern "C" fn uart_tx(rtos: G8torRtosHandle) -> ! {
    let tx_fifo_handle = unsafe { UART0_TX_FIFO.as_ref().expect("UART0 TX FIFO handle is initialized.") };
    let uart_handle = unsafe { UART0_HANDLE.as_ref().expect("UART0 handle is initialized.") };

    rtos.sleep_ms(9600);
    loop {
        let val = rtos.read_fifo(tx_fifo_handle);
        loop {
            let uart = UART0_MUTEX.get(rtos.take_mutex(uart_handle));
            let res = uart.write(val as u8);
            rtos.release_mutex(uart_handle, UART0_MUTEX.release(uart));
            match res {
                Ok(()) => break,
                Err(nb::Error::WouldBlock) => {},
            };
        }
    }
}

// extern "C" fn uart_rx(rtos: G8torRtosHandle) -> ! {
//     let rx_fifo_handle = unsafe { UART0_RX_FIFO.as_ref().expect("UART0 TX FIFO handle is initialized.") };
//     let uart_handle = unsafe { UART0_HANDLE.as_ref().expect("UART0 handle is initialized.") };

//     loop {
//         let uart = UART0_MUTEX.get(rtos.take_mutex(uart_handle));
//         let val = uart.read();
//         rtos.release_mutex(uart_handle, UART0_MUTEX.release(uart));
//         let val = match val {
//             Ok(v) => v,
//             Err(nb::Error::WouldBlock) => { continue; }
//         };
//         rtos.write_fifo(rx_fifo_handle, val as u32);
//     }
// }

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
        UART0_TX_FIFO = Some(
            inst.init_fifo()
                .expect("We haven't run out of atomics"),
        );
        UART0_RX_FIFO = Some(
            inst.init_fifo()
                .expect("We haven't run out of atomics"),
        );
        let _ = inst
            .add_thread(b"publish\0\0\0\0\0\0\0\0\0", 1, publisher)
            .expect("Failed to add red thread");
        // let _ = inst
        //     .add_thread(b"consume\0\0\0\0\0\0\0\0\0", 0, consumer)
        //     .expect("Failed to add blue thread");
        let _ = inst
            .add_thread(b"uart_tx\0\0\0\0\0\0\0\0\0", 2, uart_tx)
            .expect("Failed to add uart_tx thread");
        // let _ = inst
        //     .add_thread(b"uart_rx\0\0\0\0\0\0\0\0\0", 2, uart_rx)
        //     .expect("Failed to add uart_rx thread");
        inst.launch()
    }
}
