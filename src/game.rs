#![allow(static_mut_refs)]

use panic_halt as _;

use core::mem::MaybeUninit;

use crate::rtos::{
    self, G8torFifoHandle, G8torMutex, G8torMutexHandle, G8torRtos, G8torSemaphoreHandle,
    G8torThreadHandle,
};
use crate::{byte_str, SyncUnsafeOnceCell};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use mipidsi::{interface::SpiInterface, Builder, Display};

use embedded_hal::{digital::OutputPin, i2c::I2c};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

use tm4c123x_hal::{
    self as hal,
    adc::AdcSingle,
    gpio::{
        gpioa::{PA0, PA1, PA2, PA4, PA5, PA6, PA7},
        gpioe::{PE0, PE2, PE3},
        gpiof::{PF3, PF4},
        AlternateFunction, Analog, Floating, Input, OpenDrain, Output, PullUp, PushPull, AF1, AF2,
        AF3,
    },
    i2c::I2C,
    pac::{self, ADC0, ADC1, I2C1, SSI0, UART0},
    prelude::*,
    serial::Serial,
    spi::Spi,
};

// Standard Resources
// Single Consumer Resources
static mut JOYSTICK_X_ADC_S: MaybeUninit<AdcSingle<ADC0, PE3<Analog<Input<Floating>>>>> =
    MaybeUninit::uninit();
static mut JOYSTICK_Y_ADC_S: MaybeUninit<AdcSingle<ADC1, PE2<Analog<Input<Floating>>>>> =
    MaybeUninit::uninit();

// Shared Resources
pub static SCREEN_MUTEX: G8torMutex<
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
> = G8torMutex::empty();
pub static I2C_MUTEX: G8torMutex<
    I2C<
        I2C1,
        (
            PA6<AlternateFunction<AF3, PushPull>>,
            PA7<AlternateFunction<AF3, OpenDrain<PullUp>>>,
        ),
    >,
> = G8torMutex::empty();
pub static UART_MUTEX: G8torMutex<
    Serial<
        UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
> = G8torMutex::empty();

// Mutex Handles
pub static SCREEN_MUT: SyncUnsafeOnceCell<
    G8torMutexHandle<
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
    >,
> = SyncUnsafeOnceCell::new();
pub static I2C_MUT: SyncUnsafeOnceCell<
    G8torMutexHandle<
        I2C<
            I2C1,
            (
                PA6<AlternateFunction<AF3, PushPull>>,
                PA7<AlternateFunction<AF3, OpenDrain<PullUp>>>,
            ),
        >,
    >,
> = SyncUnsafeOnceCell::new();
pub static UART_MUT: SyncUnsafeOnceCell<
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

// FIFOs
pub static JOYSTICK_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();

// Semaphore Handles
pub static JOYSTICK_PUSH_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();
pub static BUTTONS_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();

static ON_SW1: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();
static ON_SW2: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();
static ON_SW3: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();
static ON_SW4: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();
static ON_JOYSTICK: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();

pub fn initialize(
    game_setup: fn(&mut G8torRtos),
    on_sw1: Option<fn()>,
    on_sw2: Option<fn()>,
    on_sw3: Option<fn()>,
    on_sw4: Option<fn()>,
    on_joystick: Option<fn()>,
) -> ! {
    let p = pac::Peripherals::take().unwrap();
    let mut core_p = pac::CorePeripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    // Initialize GPIO ports
    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let portd = p.GPIO_PORTD.split(&sc.power_control);
    let porte = p.GPIO_PORTE.split(&sc.power_control);
    let portf = p.GPIO_PORTF.split(&sc.power_control);

    // Enable buttons interrupt
    let mut pe4 = porte.pe4.into_pull_up_input();
    pe4.set_interrupt_mode(hal::gpio::InterruptMode::EdgeFalling);

    // Initialize Joystick ADCs
    let pe3 = porte.pe3.into_floating_input().into_analog();
    let pe2 = porte.pe2.into_floating_input().into_analog();

    let adc_x = AdcSingle::adc0(p.ADC0, pe3, &sc.power_control);
    let adc_y = AdcSingle::adc1(p.ADC1, pe2, &sc.power_control);

    // Enable Joystick Interrupt
    let mut pd2 = portd.pd2.into_pull_up_input();
    pd2.set_interrupt_mode(hal::gpio::InterruptMode::EdgeFalling);

    // Activate UART
    let uart = Serial::uart0(
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

    // Activate I2C module
    let i2c: I2C<
        I2C1,
        (
            tm4c123x_hal::gpio::gpioa::PA6<AlternateFunction<AF3, PushPull>>,
            tm4c123x_hal::gpio::gpioa::PA7<AlternateFunction<AF3, OpenDrain<PullUp>>>,
        ),
    > = I2C::<I2C1, _>::new(
        p.I2C1,
        (
            porta.pa6.into_af_push_pull::<AF3>(&mut porta.control),
            porta
                .pa7
                .into_af_open_drain::<AF3, PullUp>(&mut porta.control),
        ),
        100.khz(),
        &clocks,
        &sc.power_control,
    );

    // Activate SPI module
    let spi = Spi::<SSI0, _>::spi0(
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

    // Activate TFT Display
    let mut tft_rst = porte.pe0.into_push_pull_output();
    let tft_dc = portf.pf3.into_push_pull_output();
    let tft_cs = portf.pf4.into_push_pull_output();

    tft_rst.set_high().unwrap();
    let spi_device = ExclusiveDevice::new(spi, tft_cs, NoDelay).unwrap();
    static mut SPI_BUFFER: [u8; 16] = [0; 16];
    let di = SpiInterface::new(spi_device, tft_dc, unsafe { &mut SPI_BUFFER });

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
        .orientation(
            mipidsi::options::Orientation::new()
                .flip_horizontal()
                .flip_vertical(),
        )
        .reset_pin(tft_rst)
        .init(&mut delay_source)
        .unwrap();
    core_p.SYST = delay_source.free();
    display.clear(Rgb565::BLACK).unwrap();

    // Initialize RTOS
    let inst = unsafe { rtos::G8torRtos::new(core_p) };

    let screen_mut = inst
        .init_mutex(&SCREEN_MUTEX)
        .expect("We haven't run out of atomics");
    let i2c_mut = inst
        .init_mutex(&I2C_MUTEX)
        .expect("We haven't run out of atomics");
    let uart_mut = inst
        .init_mutex(&UART_MUTEX)
        .expect("We haven't run out of atomics");

    let joystick_fifo = inst.init_fifo().expect("We haven't run out of atomics");

    let joystick_push_sem = inst
        .init_semaphore(0)
        .expect("We haven't run out of atomics");
    let buttons_sem = inst
        .init_semaphore(0)
        .expect("We haven't run out of atomics");

    inst.add_thread(&byte_str("read_buttons"), 1, read_buttons)
        .expect("TCB list has space");
    inst.add_thread(&byte_str("read_joystick"), 1, read_joystick)
        .expect("TCB list has space");

    inst.add_periodic(100, 0, read_adc)
        .expect("Periodic TCB list has space");

    inst.add_event(pac::interrupt::GPIOD, 5, joystick_click_isr)
        .expect("Inputs are correct");
    inst.add_event(pac::interrupt::GPIOE, 5, button_isr)
        .expect("Inputs are correct");

    unsafe {
        JOYSTICK_X_ADC_S.write(adc_x);
        JOYSTICK_Y_ADC_S.write(adc_y);

        SCREEN_MUTEX.init(display);
        I2C_MUTEX.init(i2c);
        UART_MUTEX.init(uart);

        SCREEN_MUT.set(screen_mut);
        I2C_MUT.set(i2c_mut);
        UART_MUT.set(uart_mut);

        JOYSTICK_FIFO.set(joystick_fifo);

        JOYSTICK_PUSH_SEM.set(joystick_push_sem);
        BUTTONS_SEM.set(buttons_sem);

        ON_SW1.set(on_sw1);
        ON_SW2.set(on_sw2);
        ON_SW3.set(on_sw3);
        ON_SW4.set(on_sw4);
        ON_JOYSTICK.set(on_joystick);
    }

    game_setup(inst);

    unsafe { inst.launch() }
}

pub fn get_joystick() -> (f32, f32) {
    let val = rtos::read_fifo(&*JOYSTICK_FIFO);
    let x = (val & 0xFFFF) as u16; // Lower 16 bits
    let y = ((val >> 16) & 0xFFFF) as u16; // Upper 16 bits

    // Normalize to -1.0 to 1.0
    let x = -((x as f32) / (0x7FF as f32) - 1.0);
    let y = (y as f32) / (0x7FF as f32) - 1.0;

    if x.abs() < 0.1 && y.abs() < 0.1 {
        return (0.0, 0.0);
    }

    (x, y)
}

extern "C" fn read_adc() {
    // Read the joystick ADC values and push to FIFO every 100ms
    unsafe {
        let x_adc = JOYSTICK_X_ADC_S.assume_init_mut();
        let y_adc = JOYSTICK_Y_ADC_S.assume_init_mut();
        let x_val = x_adc.read();
        let y_val = y_adc.read();
        let joystick_fifo_handle = &*JOYSTICK_FIFO;
        let packed_val: u32 = ((y_val as u32) << 16) | (x_val as u32);
        rtos::write_fifo(joystick_fifo_handle, packed_val);
    }
}

// PORTD Interrupt Handler
extern "C" fn joystick_click_isr() {
    // Clear interrupt flag
    let portd = unsafe { &*pac::GPIO_PORTD::ptr() };
    portd.icr.write(|w| unsafe { w.gpio().bits(1 << 2) });

    // Signal Joystick Push Semaphore
    rtos::signal_semaphore(&*JOYSTICK_PUSH_SEM);
}

// PORTE Interrupt Handler
extern "C" fn button_isr() {
    // Clear interrupt flag
    let porte = unsafe { &*pac::GPIO_PORTE::ptr() };
    porte.icr.write(|w| unsafe { w.gpio().bits(1 << 4) });

    // Signal Buttons Semaphore
    rtos::signal_semaphore(&*BUTTONS_SEM);
}

extern "C" fn read_buttons(rtos: G8torThreadHandle) -> ! {
    let mut i2c = *I2C_MUT;
    let addr = 0x23;
    let on_sw1 = *ON_SW1;
    let on_sw2 = *ON_SW2;
    let on_sw3 = *ON_SW3;
    let on_sw4 = *ON_SW4;

    if on_sw1.is_none() && on_sw2.is_none() && on_sw3.is_none() && on_sw4.is_none() {
        // If no ISRs are set, just kill the thread
        rtos.kill();
    }

    loop {
        // Wait for button press (PCA9555 Semaphore)
        rtos::wait_semaphore(&*BUTTONS_SEM);
        rtos::sleep_ms(10); // Debounce delay
        let mut val = [0u8; 1];
        i2c.write_read(addr, &[0x00], &mut val).unwrap();
        let val = val[0];

        // SW1
        if let Some(on_sw1) = on_sw1 {
            if (val & 1 << 1) == 0 {
                on_sw1();
            }
        }

        // SW2
        if let Some(on_sw2) = on_sw2 {
            if (val & 1 << 2) == 0 {
                on_sw2();
            }
        }

        // SW3
        if let Some(on_sw3) = on_sw3 {
            if (val & 1 << 3) == 0 {
                on_sw3();
            }
        }

        // SW4
        if let Some(on_sw4) = on_sw4 {
            if (val & 1 << 4) == 0 {
                on_sw4();
            }
        }
    }
}

extern "C" fn read_joystick(rtos: G8torThreadHandle) -> ! {
    if let Some(on_joystick) = *ON_JOYSTICK {
        loop {
            // Wait for joystick push
            rtos::wait_semaphore(&*JOYSTICK_PUSH_SEM);
            on_joystick();
        }
    } else {
        // If no ISR is set, just kill the thread
        rtos.kill();
    }
}
