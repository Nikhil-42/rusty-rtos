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
    self as hal, gpio::{
        AF1, AF2, AlternateFunction, Floating, OpenDrain, Output, PushPull, gpioa::{PA2, PA4, PA5}, gpioc::{PC4, PC5}, gpioe::PE0, gpiof::{PF3, PF4}
    }, pac::{self, SSI0, UART4}, prelude::*, serial::Serial, spi::Spi
};

use cortex_m_rt::entry;
// use eh0::serial::{Read as _, Write as _};
use embedded_hal::digital::OutputPin;

use eh0::serial::Read as _;

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

static UART4_RX_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();

static UART4_MUTEX: G8torMutex<
    Serial<
        UART4,
        PC5<AlternateFunction<AF1, PushPull>>,
        PC4<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
> = G8torMutex::empty();

extern "C" fn uart_rx(_rtos: G8torThreadHandle) -> ! {
    let rx_fifo_handle = &*UART4_RX_FIFO;
    let uart_handle = &*UART4_MUT;

    loop {
        let uart = UART4_MUTEX.get(rtos::take_mutex(uart_handle));
        let val = uart.read();
        rtos::release_mutex(uart_handle, UART4_MUTEX.release(uart));
        let val = match val {
            Ok(v) => v,
            Err(nb::Error::WouldBlock) => {
                continue;
            }
        };
        rtos::write_fifo(rx_fifo_handle, val as u32);
    }
}

extern "C" fn draw_rect(_rtos: G8torThreadHandle) -> ! {
    let uart_rx = &*UART4_RX_FIFO;
    let display = unsafe { &mut *SCREEN_S.as_mut_ptr() };

    loop {
        let x = rtos::read_fifo(uart_rx);
        let y = rtos::read_fifo(uart_rx);
        let width = rtos::read_fifo(uart_rx);
        let height = rtos::read_fifo(uart_rx);
        let r = rtos::read_fifo(uart_rx) as u8;
        let g = rtos::read_fifo(uart_rx) as u8;
        let b = rtos::read_fifo(uart_rx) as u8;

        let color = Rgb565::new(r, g, b);

        let rect = Rectangle::new(
            Point::new(x as i32, y as i32 + 20),
            Size::new(width as u32, height as u32),
        );

        let style = PrimitiveStyle::with_stroke(color, 1);
        rect.draw_styled(&style, display).unwrap();
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

    // Activate UART
    let uart = hal::serial::Serial::uart4(
        p.UART4,
        portc
            .pc5
            .into_af_push_pull::<hal::gpio::AF1>(&mut portc.control),
        portc
            .pc4
            .into_af_push_pull::<hal::gpio::AF1>(&mut portc.control),
        (),
        (),
        115200_u32.bps(),
        hal::serial::NewlineMode::SwapLFtoCRLF,
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
        .orientation(mipidsi::options::Orientation::new().flip_horizontal())
        .reset_pin(tft_rst)
        .init(&mut delay_source)
        .unwrap();
    // 240 x 320 resolution
    core_p.SYST = delay_source.free();
    display.clear(Rgb565::BLACK).unwrap();

    let inst = unsafe { rtos::G8torRtos::new(core_p) };

    let uart0_handle = inst
        .init_mutex(&UART4_MUTEX)
        .expect("We haven't run out of atomics");
    let uart0_rx_fifo = inst.init_fifo().expect("We haven't run out of atomics");

    inst.add_thread(&byte_str("uart_rx"), 2, uart_rx)
        .expect("Failed to add uart_rx thread");
    inst.add_thread(&byte_str("draw_rect"), 2, draw_rect)
        .expect("Failed to add draw_rect thread");

    unsafe {
        SCREEN_S.as_mut_ptr().write(display);

        UART4_MUTEX.init(uart);
        UART4_MUT.set(uart0_handle);

        UART4_RX_FIFO.set(uart0_rx_fifo);

        inst.launch()
    }
}
