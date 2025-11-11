#![no_std]
#![no_main]

use panic_semihosting as _;

use core::{fmt::Write};
use bme280::i2c::BME280;
use bmi160::Bmi160;
use opt300x::Opt300x;
use cortex_m_rt::entry;
use tm4c123x_hal::{self as hal, delay::Delay, gpio::{PullUp, AF1, AF3}, i2c::I2C, pac::{self, I2C1}, prelude::*, serial::Serial};

use embedded_hal_bus::{i2c::AtomicDevice, util::AtomicCell};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let core_p = pac::CorePeripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();
    let mut delay_provider = Delay::new(core_p.SYST, &clocks);

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

    let i2c_atomic = AtomicCell::new(i2c);
    let mut opt3001 = Opt300x::new_opt3001(AtomicDevice::new(&i2c_atomic), opt300x::SlaveAddr::Alternative(true, true) );
    let mut bmi160 = Bmi160::new_with_i2c(AtomicDevice::new(&i2c_atomic), bmi160::SlaveAddr::Alternative(true) );
    // let mut bme280 = BME280::new_secondary(AtomicDevice::new(&i2c_atomic));

    // Light Sensor (OPT3001)
    let opt3001_id = opt3001.get_device_id().unwrap();

    // Motion Sensor (BMI160)
    let bmi160_id = bmi160.chip_id().unwrap();

    // Environmental Sensor (BMM150)
    // let address = 0x13;
    // let mut buf = [0u8; 1];
    // i2c.write(address, &[0x4B, 0x01]).unwrap();
    // i2c.write_read(address, &[0x40], &mut buf).unwrap();
    // let device_id = u32::from(buf[0]);

    // let bmm150 = device_id;

    // Pressure Sensor (BME280)
    // bme280.init(&mut delay_provider).unwrap();

    // Activate UART
    let mut uart = Serial::uart0(
        p.UART0,
        porta
            .pa1
            .into_af_push_pull::<AF1>(&mut porta.control),
        porta
            .pa0
            .into_af_push_pull::<AF1>(&mut porta.control),
        (),
        (),
        115200_u32.bps(),
        hal::serial::NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control,
    );

    loop {
        writeln!(uart, "OPT3001: {:#06X}", opt3001_id).unwrap();
        // writeln!(uart, "BMM150: {:#06X}", bmm150).unwrap();
        writeln!(uart, "BMI160: {:#06X}", bmi160_id).unwrap();
        // writeln!(uart, "BME280: {:#06X}", bme280).unwrap();
        cortex_m::asm::delay(500_000);
    }
}
