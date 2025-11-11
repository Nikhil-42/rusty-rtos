#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::mem::MaybeUninit;

use eel4745c::{
    byte_str,
    rtos::{self, G8torFifoHandle, G8torMutex, G8torMutexHandle, G8torThreadHandle},
    SyncUnsafeOnceCell,
};
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle, StyledDrawable},
};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use mipidsi::{interface::SpiInterface, Builder, Display};
use panic_halt as _;

use tm4c123x_hal::{
    self as hal,
    gpio::{
        AF1, AF2, AlternateFunction, Floating, OpenDrain, Output, PushPull, gpioa::{PA0, PA1, PA2, PA4, PA5}, gpioc::{PC4, PC5, PC6, PC7}, gpioe::PE0, gpiof::{PF3, PF4}
    },
    pac::{self, SSI0, UART0, UART3, UART4},
    prelude::*,
    serial::Serial,
    spi::Spi,
};

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use eh0::serial::{Read as _, Write as _};

static mut SCREEN_S: MaybeUninit<
    Display<
        SpiInterface<
            '_,
            ExclusiveDevice<
                Spi<
                    SSI0,
                    (
                        PA2<AlternateFunction<AF2, PushPull>>,
                        PA4<AlternateFunction<AF2, OpenDrain<Floating>>>,
                        PA5<AlternateFunction<AF2, PushPull>>,
                    ),
                >,
                PF4<Output<PushPull>>,
                NoDelay,
            >,
            PF3<Output<PushPull>>,
        >,
        mipidsi::models::ST7789,
        PE0<Output<PushPull>>,
    >,
> = MaybeUninit::uninit();

static UART4_RX_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();
static UART3_RX_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();
static UART3_TX_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();
static UART0_TX_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();

static UART4_MUT: SyncUnsafeOnceCell<
    G8torMutexHandle<
        Serial<
            UART4,
            PC5<AlternateFunction<AF1, PushPull>>,
            PC4<AlternateFunction<AF1, PushPull>>,
            (),
            (),
        >,
    >,
> = SyncUnsafeOnceCell::new();
static UART4_MUTEX: G8torMutex<
    Serial<
        UART4,
        PC5<AlternateFunction<AF1, PushPull>>,
        PC4<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
> = G8torMutex::empty();

static UART3_MUT: SyncUnsafeOnceCell<
    G8torMutexHandle<
        Serial<
            UART3,
            PC7<AlternateFunction<AF1, PushPull>>,
            PC6<AlternateFunction<AF1, PushPull>>,
            (),
            (),
        >,
    >,
> = SyncUnsafeOnceCell::new();
static UART3_MUTEX: G8torMutex<
    Serial<
        UART3,
        PC7<AlternateFunction<AF1, PushPull>>,
        PC6<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
> = G8torMutex::empty();

static UART0_MUT: SyncUnsafeOnceCell<
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
static UART0_MUTEX: G8torMutex<
    Serial<
        UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
> = G8torMutex::empty();

extern "C" fn beagle_uart_rx(_rtos: G8torThreadHandle) -> ! {
    let rx_fifo_handle = &*UART4_RX_FIFO;
    let mut uart = &*UART4_MUT;

    // loop {
    //     // Forward to debug uart
    //     let val = uart_handle.read().unwrap();
    //     debug_uart.write(val).unwrap();
    // }
    'restart: loop {
        for _ in 0..3 {
            match uart.read() {
                Ok(v) if v == 0xFF => {},
                _ => continue 'restart,
            }
        }

        // 3 consecutive 0xFF received, read the length
        let length = uart.read().unwrap() as usize;
        rtos::write_fifo(rx_fifo_handle, length as u32);

        for _ in 0..length {
            // x, y, w, h
            for _ in 0..4 {
                let mut float_buff = [0u8; 4];
                // Read 4 bytes for f32
                for i in 0..4 {
                    float_buff[i] = uart.read().unwrap();
                }
                rtos::write_fifo(rx_fifo_handle, u32::from_be_bytes(float_buff));
            }
        }
    }
}

// extern "C" fn nrf_uart_rx(_rtos: G8torThreadHandle) -> ! {
//     let rx_fifo_handle = &*UART3_RX_FIFO;
//     let uart_handle = &*UART3_MUT;

//     loop {
//         let uart = UART3_MUTEX.get(rtos::take_mutex(uart_handle));
//         let val = uart.read();
//         rtos::release_mutex(uart_handle, UART3_MUTEX.release(uart));
//         let val = match val {
//             Ok(v) => v,
//             Err(nb::Error::WouldBlock) => {
//                 continue;
//             }
//         };
//         rtos::write_fifo(rx_fifo_handle, val as u32);
//     }
// }

// extern "C" fn nrf_uart_tx(_rtos: G8torThreadHandle) -> ! {
//     let rx_fifo_handle = &*UART3_TX_FIFO;
//     let uart_handle = &*UART3_MUT;

//     loop {
//         let val = rtos::read_fifo(rx_fifo_handle);

//         let uart = UART3_MUTEX.get(rtos::take_mutex(uart_handle));
//         uart.write_all(val.to_le_bytes().as_slice());
//         rtos::release_mutex(uart_handle, UART3_MUTEX.release(uart));
//     }
// }

extern "C" fn draw_rect(_rtos: G8torThreadHandle) -> ! {
    let uart_rx = &*UART4_RX_FIFO;
    let display = unsafe { &mut *SCREEN_S.as_mut_ptr() };

    const WIDTH: f32 = 240f32;
    const HEIGHT: f32 = 320f32;

    const COLORS: [Rgb565; 8] = [
        Rgb565::RED,
        Rgb565::GREEN,
        Rgb565::BLUE,
        Rgb565::YELLOW,
        Rgb565::CYAN,
        Rgb565::MAGENTA,
        Rgb565::WHITE,
        Rgb565::RED,
    ];

    let mut num_faces = 0usize;
    let mut bounding_boxes = [Rectangle::default(); 8]; // Max 8 faces

    loop {
        for i in 0usize..num_faces {
            let style = PrimitiveStyle::with_stroke(Rgb565::BLACK, 1);
            bounding_boxes[i].draw_styled(&style, display).unwrap();
        }

        num_faces = rtos::read_fifo(uart_rx).min(8) as usize;

        for i in 0usize..(num_faces.min(8) as usize) {
            let x = f32::from_bits(rtos::read_fifo(uart_rx));
            let y = f32::from_bits(rtos::read_fifo(uart_rx));
            let w = f32::from_bits(rtos::read_fifo(uart_rx));
            let h = f32::from_bits(rtos::read_fifo(uart_rx));

            bounding_boxes[i] = Rectangle::new(
                Point::new((x * WIDTH) as i32, (y * HEIGHT) as i32),
                Size::new((w * WIDTH) as u32, (h * HEIGHT) as u32),
            );

            let style = PrimitiveStyle::with_stroke(COLORS[i], 1);
            bounding_boxes[i].draw_styled(&style, display).unwrap();
        }

        rtos::sleep_ms(100);
    }
}

extern "C" fn debug_tx(_rtos: G8torThreadHandle) -> ! {
    let tx_fifo_handle = &*UART0_TX_FIFO;
    let mut uart_handle = &*UART0_MUT;

    loop {
        let val = rtos::read_fifo(tx_fifo_handle);

        uart_handle.write(val as u8).unwrap();
    }
}

static mut SPI_BUFFER: [u8; 256] = [0; 256];

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
    let mut portc = p.GPIO_PORTC.split(&sc.power_control);
    let porte = p.GPIO_PORTE.split(&sc.power_control);
    let portf = p.GPIO_PORTF.split(&sc.power_control);

    // Activate UART4
    let uart4 = hal::serial::Serial::uart4(
        p.UART4,
        portc
            .pc5
            .into_af_push_pull::<hal::gpio::AF1>(&mut portc.control),
        portc
            .pc4
            .into_af_push_pull::<hal::gpio::AF1>(&mut portc.control),
        (),
        (),
        9600_u32.bps(),
        hal::serial::NewlineMode::Binary,
        &clocks,
        &sc.power_control,
    );

    // Activate UART3
    let uart3 = hal::serial::Serial::uart3(
        p.UART3,
        portc
            .pc7
            .into_af_push_pull::<hal::gpio::AF1>(&mut portc.control),
        portc
            .pc6
            .into_af_push_pull::<hal::gpio::AF1>(&mut portc.control),
        (),
        (),
        115200_u32.bps(),
        hal::serial::NewlineMode::Binary,
        &clocks,
        &sc.power_control,
    );

    // Activate UART0
    let uart0 = hal::serial::Serial::uart0(
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
        hal::serial::NewlineMode::Binary,
        &clocks,
        &sc.power_control,
    );

    // Activate SPI module
    let spi = hal::spi::Spi::<SSI0, _>::spi0(
        p.SSI0,
        (
            porta
                .pa2
                .into_af_push_pull::<hal::gpio::AF2>(&mut porta.control),
            porta
                .pa4
                .into_af_open_drain::<hal::gpio::AF2, Floating>(&mut porta.control),
            porta
                .pa5
                .into_af_push_pull::<hal::gpio::AF2>(&mut porta.control),
        ),
        hal::spi::MODE_3,
        20.mhz(),
        &clocks,
        &sc.power_control,
    );

    let mut tft_rst = porte.pe0.into_push_pull_output();
    tft_rst.set_high().unwrap();
    let tft_dc = portf.pf3.into_push_pull_output();
    let tft_cs = portf.pf4.into_push_pull_output();

    let spi_device = ExclusiveDevice::new(spi, tft_cs, NoDelay).unwrap();
    let di = SpiInterface::new(spi_device, tft_dc, unsafe { &mut SPI_BUFFER });

    let mut core_p = pac::CorePeripherals::take().unwrap();

    let mut delay_source = hal::delay::Delay::new(core_p.SYST, &clocks);
    let mut display: Display<
        SpiInterface<
            '_,
            ExclusiveDevice<
                Spi<
                    SSI0,
                    (
                        PA2<AlternateFunction<AF2, PushPull>>,
                        PA4<AlternateFunction<AF2, OpenDrain<Floating>>>,
                        PA5<AlternateFunction<AF2, PushPull>>,
                    ),
                >,
                PF4<Output<PushPull>>,
                NoDelay,
            >,
            PF3<Output<PushPull>>,
        >,
        mipidsi::models::ST7789,
        PE0<Output<PushPull>>,
    > = Builder::new(mipidsi::models::ST7789, di)
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .orientation(mipidsi::options::Orientation::new().flip_horizontal().flip_vertical())
        .reset_pin(tft_rst)
        .init(&mut delay_source)
        .unwrap();
    // 240 x 320 resolution
    core_p.SYST = delay_source.free();
    display.clear(Rgb565::BLACK).unwrap();

    let inst = unsafe { rtos::G8torRtos::new(core_p) };

    let uart4_handle = inst
        .init_mutex(&UART4_MUTEX)
        .expect("We haven't run out of atomics");
    let uart3_handle = inst
        .init_mutex(&UART3_MUTEX)
        .expect("We haven't run out of atomics");
    let uart0_handle = inst
        .init_mutex(&UART0_MUTEX)
        .expect("We haven't run out of atomics");

    let uart4_rx_fifo = inst.init_fifo().expect("We haven't run out of atomics");
    let uart3_tx_fifo = inst.init_fifo().expect("We haven't run out of atomics");
    let uart3_rx_fifo = inst.init_fifo().expect("We haven't run out of atomics");
    let uart0_tx_fifo = inst.init_fifo().expect("We haven't run out of atomics");

    inst.add_thread(&byte_str("beagle_uart_rx"), 2, beagle_uart_rx)
        .expect("Failed to add uart_rx thread");
    // inst.add_thread(&byte_str("nrf_uart_rx"), 2, nrf_uart_rx)
    //     .expect("Failed to add uart_rx thread");
    // inst.add_thread(&byte_str("nrf_uart_rx"), 2, nrf_uart_tx)
    //     .expect("Failed to add uart_rx thread");
    inst.add_thread(&byte_str("draw_rect"), 2, draw_rect)
        .expect("Failed to add draw_rect thread");
    inst.add_thread(&byte_str("debug_tx"), 2, debug_tx)
        .expect("Failed to add debug_tx thread");

    unsafe {
        SCREEN_S.as_mut_ptr().write(display);

        UART4_MUTEX.init(uart4);
        UART4_MUT.set(uart4_handle);

        UART3_MUTEX.init(uart3);
        UART3_MUT.set(uart3_handle);

        UART0_MUTEX.init(uart0);
        UART0_MUT.set(uart0_handle);

        UART4_RX_FIFO.set(uart4_rx_fifo);
        UART3_RX_FIFO.set(uart3_rx_fifo);
        UART3_TX_FIFO.set(uart3_tx_fifo);
        UART0_TX_FIFO.set(uart0_tx_fifo);

        inst.launch()
    }
}
