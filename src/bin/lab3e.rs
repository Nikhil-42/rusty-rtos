#![no_std]
#![no_main]

use bmi160::{interface::I2cInterface, Bmi160};
use eel4745c::rtos::{self, G8torAtomic};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_bus::{i2c::AtomicDevice, util::AtomicCell};
use opt300x::{ic::Opt3001, mode::Continuous, Opt300x};
use panic_halt as _;

use tm4c123x_hal::{self as hal, gpio::{gpioa::{PA0, PA1, PA6, PA7}, gpiof::{PF0, PF1, PF2, PF3, PF4}, AlternateFunction, Input, OpenDrain, Output, PullUp, PushPull, AF1, AF3, AF6, AF7}, i2c::I2C, pac::{self, I2C1, TIMER0, TIMER1, UART0}, prelude::*, pwm::{EvenPWM, OddPWM}, serial::Serial};
use tm4c123x_hal::pwm::Timer;
use eh0::Pwm;

use cortex_m_rt::entry;
use core::{fmt::Write, mem::MaybeUninit};

static mut I2C_BUS: MaybeUninit<AtomicCell<I2C<I2C1, (PA6<AlternateFunction<AF3, PushPull>>, PA7<AlternateFunction<AF3, OpenDrain<PullUp>>>)>>> = MaybeUninit::uninit();
static UART_SEM: G8torAtomic = G8torAtomic::new(1);
static IMU_SEM: G8torAtomic = G8torAtomic::new(1);
static mut UART0_S: Option<Serial<UART0, PA1<AlternateFunction<AF1, PushPull>>, PA0<AlternateFunction<AF1, PushPull>>, (), ()>> = None;
static mut SW1_S: Option<PF4<Input<PullUp>>> = None;
static mut SW2_S: Option<PF0<Input<PullUp>>> = None;
static mut OPT3001_S: Option<Opt300x<AtomicDevice<I2C<I2C1, (PA6<AlternateFunction<AF3, PushPull>>, PA7<AlternateFunction<AF3, OpenDrain<PullUp>>>)>>, Opt3001, Continuous>> = None;
static mut BMI160_S: Option<Bmi160<I2cInterface<AtomicDevice<I2C<I2C1, (PA6<AlternateFunction<AF3, PushPull>>, PA7<AlternateFunction<AF3, OpenDrain<PullUp>>>)>>>>> = None;
static mut R_LED_S: Option<OddPWM<TIMER0>> = None;
static mut G_LED_S: Option<OddPWM<TIMER1>> = None;
static mut B_LED_S: Option<EvenPWM<TIMER1>> = None;


// Thread0 - get x-axis of accelerometer, shine
// blue LED according to x-axis magnitude.
extern "C" fn poll_accelx() -> ! {
    let mut b_led = unsafe { B_LED_S.take() }.expect("Blue LED is initialized.");
    b_led.set_period(u8::max_value());
    b_led.enable(());
    let selector = bmi160::SensorSelector::new().accel();

    loop {
        IMU_SEM.acquire();
        let imu = unsafe { BMI160_S.as_mut().unwrap_unchecked() };
        let data = imu.data(selector);
        IMU_SEM.release();

        let x = match data {
            Err(_) => {
                // In case of error, try again later
                cortex_m::asm::delay(1_000_000 / 12);
                continue;
            },
            Ok(data) => data.accel.unwrap().x.abs() as u16,
        };
        let x = ((x >> 8) as u32).min(u8::max_value().into());

        b_led.set_duty((), x);

        cortex_m::asm::delay(1_000_000 / 12 + 1_000);
    }
}

// Thread1 - get y-axis gyroscope, shine red
// LED according to y-axis magnitude.
extern "C" fn poll_gyroy() -> ! {
    let mut r_led = unsafe { R_LED_S.take() }.expect("Red LED is initialized.");
    r_led.set_period(u8::max_value());
    r_led.enable(());
    let selector = bmi160::SensorSelector::new().gyro();

    loop {
        // make sure only one thread accesses the IMU at a time
        IMU_SEM.acquire();
        let imu = unsafe { BMI160_S.as_mut().unwrap_unchecked() };
        let data = imu.data(selector);
        IMU_SEM.release();

        let y = match data {
            Err(_) => {
                // In case of error, try again later
                cortex_m::asm::delay(1_000_000 / 12);
                continue;
            },
            Ok(data) => data.gyro.unwrap().y.abs() as u16,

        };
        let y = ((y >> 8) as u32).min(u8::max_value().into()); 

        r_led.set_duty((), y);

        cortex_m::asm::delay(1_000_000 / 12 + 2_000);
    }
}

// Thread2 - get optical sensor data, shine
// green LED according to light magnitude.
extern "C" fn poll_optical() -> ! {
    let mut g_led = unsafe { G_LED_S.take().expect("Green LED is initialized.") };
    g_led.set_period(u8::max_value());
    g_led.enable(());
    // Atomic bus already handles concurrency
    let mut opt3001 = unsafe { OPT3001_S.as_mut().unwrap_unchecked() };

    loop {
        let raw = match opt3001.read_raw() {
            Err(_) => {
                // In case of error, try again later
                cortex_m::asm::delay(1_000_000 / 12);
                continue;
            },
            Ok((_, raw)) => raw,
        };
        let raw = (raw >> 7).min(u8::max_value().into());

        g_led.set_duty((), u32::from(raw));

        cortex_m::asm::delay(1_000_000 / 12 + 3_000);
    }
}

// Thread3 - Poll button 1, output a message
// through the UART terminal if pressed.
extern "C" fn poll_sw1() -> ! {
    let mut sw1 = unsafe { SW1_S.take() }.expect("SW1 is initialized.");

    loop {
        if sw1.is_low().unwrap() {
            UART_SEM.acquire();
            unsafe {
                writeln!(UART0_S.as_mut().unwrap_unchecked(), "SW1 was pressed!").unwrap();
            }
            UART_SEM.release();
            cortex_m::asm::delay(1_000_000 / 12);
        } 
    }
}

// Thread4 - Poll button 2, output a message
// through the UART terminal if pressed.
extern "C" fn poll_sw2() -> ! {
    let mut sw2 = unsafe { SW2_S.take() }.expect("SW1 is initialized.");

    loop {
        if sw2.is_low().unwrap() {
            UART_SEM.acquire();
            unsafe {
                writeln!(UART0_S.as_mut().unwrap_unchecked(), "SW2 was pressed!").unwrap();
            }
            UART_SEM.release();
            cortex_m::asm::delay(1_000_000 / 12);
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
    
    let i2c_atomic = unsafe { I2C_BUS.write(AtomicCell::new(i2c)) };
    
    let opt3001 = Opt300x::new_opt3001(AtomicDevice::new(i2c_atomic), opt300x::SlaveAddr::Alternative(true, true) );
    let mut bmi160 = Bmi160::new_with_i2c(AtomicDevice::new(i2c_atomic), bmi160::SlaveAddr::Alternative(true) );

    let mut opt3001 = match opt3001.into_continuous() {
        Ok(sensor) => sensor,
        Err(_) => panic!("Failed to configure OPT3001"),
    };
    opt3001.set_lux_range(opt300x::LuxRange::Manual(0b0110)).unwrap();
    opt3001.set_integration_time(opt300x::IntegrationTime::Ms100).unwrap();
    opt3001.enable_exponent_masking().unwrap();

    bmi160.set_accel_power_mode(bmi160::AccelerometerPowerMode::Normal).unwrap();
    bmi160.set_gyro_power_mode(bmi160::GyroscopePowerMode::Normal).unwrap();    
    let selector = bmi160::SensorSelector::new().accel();

    let mut portf = p.GPIO_PORTF.split(&sc.power_control);

    let mut timer0 = Timer::timer0(&sc.power_control, p.TIMER0);
    let mut timer1 = Timer::timer1(&sc.power_control, p.TIMER1);

    let mut r_pwm = timer0.into_odd(portf.pf1.into_af_push_pull::<AF7>(&mut portf.control));
    let (mut b_pwm, mut  g_pwm) = timer1.into_both(
        portf.pf2.into_af_push_pull::<AF7>(&mut portf.control),
        portf.pf3.into_af_push_pull::<AF7>(&mut portf.control)
    );

    unsafe {
        // Insert Resources
        UART0_S.insert(uart);
        SW1_S.insert(portf.pf4.into_pull_up_input());
        SW2_S.insert(portf.pf0.unlock(&mut portf.control).into_pull_up_input());
        OPT3001_S.insert(opt3001);
        BMI160_S.insert(bmi160);
        R_LED_S.insert(r_pwm);
        G_LED_S.insert(g_pwm);
        B_LED_S.insert(b_pwm);
    }

    unsafe {
        let inst = rtos::G8torRtos::get(pac::CorePeripherals::take().unwrap());
        let accelx_thread = inst.add_thread(poll_accelx).expect("Failed to add blue thread");
        let gyroy_thread = inst.add_thread(poll_gyroy).expect("Failed to add red thread");
        let optical_thread = inst.add_thread(poll_optical).expect("Failed to add green thread");
        let sw1_thread = inst.add_thread(poll_sw1).expect("Failed to add red thread");
        let sw2_thread = inst.add_thread(poll_sw2).expect("Failed to add blue thread");
        
        inst.launch()
    }
}
