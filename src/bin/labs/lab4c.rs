#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use eel4745c::{
    rtos::{self, G8torFifoHandle, G8torMutex, G8torMutexHandle, G8torThreadHandle},
    SyncUnsafeOnceCell,
};
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
use eh0::serial::{Read as _, Write as _};

static mut R_LED_S: SyncUnsafeOnceCell<PF1<Output<PushPull>>> = SyncUnsafeOnceCell::new();
static mut B_LED_S: SyncUnsafeOnceCell<PF2<Output<PushPull>>> = SyncUnsafeOnceCell::new();

static UART0_HANDLE: SyncUnsafeOnceCell<
    G8torMutexHandle<
        Serial<
            UART0,
            PA1<AlternateFunction<AF1, PushPull>>,
            PA0<AlternateFunction<AF1, PushPull>>,
            (),
            (),
        >,
    >,
> = SyncUnsafeOnceCell::new();
static UART0_TX_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();
static UART0_RX_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();
static UART0_MUTEX: G8torMutex<
    Serial<
        UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
> = G8torMutex::empty();

extern "C" fn publisher(rtos: G8torThreadHandle) -> ! {
    let tx_fifo_handle = &*UART0_TX_FIFO;

    for _ in 0..20 {
        rtos::write_fifo(tx_fifo_handle, rtos.tid() as u32);
        rtos::sleep_ms(1_000);
    }

    rtos.kill();
}

extern "C" fn consumer(_rtos: G8torThreadHandle) -> ! {
    let rx_fifo_handle = &*UART0_RX_FIFO;

    loop {
        let val = rtos::read_fifo(rx_fifo_handle);
        if rtos::kill_thread(val as usize) == 1 {
            // Failed to kill thread
            // Spawn a new publisher thread
            let mut buff = *b"publish\0\0\0\0\0\0\0\0\0";
            buff[7] = val as u8 + b'0';
            rtos::spawn_thread(&buff, 2, publisher);
        }
    }
}

extern "C" fn uart_tx(_rtos: G8torThreadHandle) -> ! {
    let tx_fifo_handle = &*UART0_TX_FIFO;
    let uart_handle = &*UART0_HANDLE;

    loop {
        let val = rtos::read_fifo(tx_fifo_handle);
        loop {
            let uart = UART0_MUTEX.get(rtos::take_mutex(uart_handle));
            let res = uart.write(val as u8);
            rtos::release_mutex(uart_handle, UART0_MUTEX.release(uart));
            match res {
                Ok(()) => break,
                Err(nb::Error::WouldBlock) => {}
            };
        }
    }
}

extern "C" fn uart_rx(_rtos: G8torThreadHandle) -> ! {
    let rx_fifo_handle = &*UART0_RX_FIFO;
    let uart_handle = &*UART0_HANDLE;

    loop {
        let uart = UART0_MUTEX.get(rtos::take_mutex(uart_handle));
        let val = uart.read();
        rtos::release_mutex(uart_handle, UART0_MUTEX.release(uart));
        let val = match val {
            Ok(v) => v,
            Err(nb::Error::WouldBlock) => {
                continue;
            }
        };
        rtos::write_fifo(rx_fifo_handle, val as u32);
    }
}

extern "C" fn periodic_task() {
    let tx_fifo_handle = &*UART0_TX_FIFO;
    rtos::write_fifo(tx_fifo_handle, 0xff);
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

    let inst = unsafe { rtos::G8torRtos::new(pac::CorePeripherals::take().unwrap()) };

    let uart0_handle = inst
        .init_mutex(&UART0_MUTEX)
        .expect("We haven't run out of atomics");
    let uart0_tx_fifo = inst.init_fifo().expect("We haven't run out of atomics");
    let uart0_rx_fifo = inst.init_fifo().expect("We haven't run out of atomics");

    inst.add_thread(b"consume\0\0\0\0\0\0\0\0\0", 0, consumer)
        .expect("Failed to add subscriber thread");
    inst.add_thread(b"uart_tx\0\0\0\0\0\0\0\0\0", 2, uart_tx)
        .expect("Failed to add uart_tx thread");
    inst.add_thread(b"uart_rx\0\0\0\0\0\0\0\0\0", 2, uart_rx)
        .expect("Failed to add uart_rx thread");
    inst.add_thread(b"publish\0\0\0\0\0\0\0\0\0", 1, publisher)
        .expect("Failed to add publisher thread");
    let _ = inst
        .add_periodic(2000, 0, periodic_task)
        .expect("There is space in the Periodic Threads LL");

    unsafe {
        R_LED_S.set(portf.pf1.into_push_pull_output());
        B_LED_S.set(portf.pf2.into_push_pull_output());
        UART0_MUTEX.init(uart);
        UART0_HANDLE.set(uart0_handle);
        UART0_TX_FIFO.set(uart0_tx_fifo);
        UART0_RX_FIFO.set(uart0_rx_fifo);
        inst.launch()
    }
}
