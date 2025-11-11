#![no_std]
#![no_main]

use bmi160::Bmi160;
use embedded_hal_bus::{i2c::AtomicDevice, util::AtomicCell};
use opt300x::Opt300x;
use panic_semihosting as _;

use core::{fmt::Write};
use cortex_m_rt::entry;
use tm4c123x_hal::{self as hal, gpio::{PullUp, AF3}, i2c::I2C, pac::{self, I2C1}, prelude::*};

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

    loop {
        writeln!(uart, "BMI160: {:6}", bmi160.data(selector).unwrap().accel.unwrap().x).unwrap();
        let raw = opt3001.read_raw().unwrap();
        let raw = (raw.1 as u32) << (raw.0 as u32);
        writeln!(uart, "OPT3001: {:6}", raw).unwrap();
        cortex_m::asm::delay(500_000);
    }
}
