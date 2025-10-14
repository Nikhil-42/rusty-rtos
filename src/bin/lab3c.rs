#![no_std]
#![no_main]

use bmi160::Bmi160;
use eel4745c::rtos::{self, Semaphore};
use embedded_hal::digital::InputPin;
use embedded_hal_bus::{i2c::AtomicDevice, util::AtomicCell};
use opt300x::{ic::Opt3001, mode::Continuous, Opt300x};
use panic_halt as _;

use tm4c123x_hal::{self as hal, gpio::{gpioa::{PA0, PA1}, gpiof::{PF0, PF1, PF2, PF3, PF4}, AlternateFunction, Input, OpenDrain, OpenDrainMode, Output, PullUp, PushPull, AF1, AF3}, i2c::I2C, pac::{self, I2C1, UART0}, prelude::*, serial::Serial};

use cortex_m_rt::entry;
use core::fmt::Write;

static UART_SEM: Semaphore = Semaphore::new(1);
static mut UART0_S: Option<Serial<UART0, PA1<AlternateFunction<AF1, PushPull>>, PA0<AlternateFunction<AF1, PushPull>>, (), ()>> = None;
static mut SW1_S: Option<PF4<Input<PullUp>>> = None;
static mut SW2_S: Option<PF0<Input<PullUp>>> = None;
static mut OPT3001_S: Option<Opt300x<AtomicDevice<I2C<I2C1, (PF0<Output<PushPull>>, PF0<Output<PushPull>>)>>, Opt3001, Continuous>> = None;
static mut BMI160_S: Option<Bmi160<AtomicDevice<I2C<I2C1, (PF0<Output<PushPull>>, PF0<Output<PushPull>>)>>>> = None;
static mut R_LED_S: Option<PF1<Output<PushPull>>> = None;
static mut G_LED_S: Option<PF3<Output<PushPull>>> = None;
static mut B_LED_S: Option<PF2<Output<PushPull>>> = None;

extern "C" fn poll_sw1() -> ! {
    let mut sw1 = unsafe { SW1_S.take() }.expect("SW1 is initialized.");

    loop {
        if sw1.is_low().unwrap() {
            UART_SEM.acquire();
            unsafe {
                writeln!(UART0_S.as_mut().unwrap_unchecked(), "SW1 was pressed!").unwrap();
            }
            UART_SEM.release();
            cortex_m::asm::delay(100_000_000 / 12);
        } 
    }
}

extern "C" fn poll_sw2() -> ! {
    let mut sw2 = unsafe { SW2_S.take() }.expect("SW1 is initialized.");

    loop {
        if sw2.is_low().unwrap() {
            UART_SEM.acquire();
            unsafe {
                writeln!(UART0_S.as_mut().unwrap_unchecked(), "SW2 was pressed!").unwrap();
            }
            UART_SEM.release();
            cortex_m::asm::delay(100_000_000 / 12);
        } 
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

    // Activate I2C
    let i2c = I2C::<I2C1, _>::new(
        p.I2C1,
        (
            porta.pa6.into_af_push_pull::<AF3>(&mut porta.control),
            porta.pa7.into_af_open_drain::<AF3, PullUp>(&mut porta.control)
        ),
        100.khz(),
        &clocks,
        &sc.power_control,
    );
    
    let i2c_atomic = AtomicCell::new(i2c);
    let opt3001 = Opt300x::new_opt3001(AtomicDevice::new(&i2c_atomic), opt300x::SlaveAddr::Alternative(true, true) );
    let mut bmi160 = Bmi160::new_with_i2c(AtomicDevice::new(&i2c_atomic), bmi160::SlaveAddr::Alternative(true) );

    let mut opt3001 = match opt3001.into_continuous() {
        Ok(sensor) => sensor,
        Err(_) => panic!("Failed to configure OPT3001"),
    };
    bmi160.set_accel_power_mode(bmi160::AccelerometerPowerMode::Normal).unwrap();
    bmi160.set_gyro_power_mode(bmi160::GyroscopePowerMode::Normal).unwrap();    
    let selector = bmi160::SensorSelector::new().accel();

    let mut portf = p.GPIO_PORTF.split(&sc.power_control);

    unsafe {
        // Insert Resources
        SW1_S.insert(portf.pf4.into_pull_up_input());
        SW2_S.insert(portf.pf0.unlock(&mut portf.control).into_pull_up_input());

        R_LED_S.insert(portf.pf1.into_push_pull_output());
        G_LED_S.insert(portf.pf3.into_push_pull_output());
        B_LED_S.insert(portf.pf2.into_push_pull_output());

        UART0_S.insert(uart);
    }



    unsafe {
        let inst = rtos::G8torRtos::new(pac::CorePeripherals::take().unwrap());
        let sw1_thread = inst.add_thread(poll_sw1).expect("Failed to add red thread");
        let sw2_thread = inst.add_thread(poll_sw2).expect("Failed to add blue thread");
        inst.launch()
    }
}
