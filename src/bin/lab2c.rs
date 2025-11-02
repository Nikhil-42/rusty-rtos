#![no_std]
#![no_main]

use embedded_graphics::mono_font::MonoTextStyleBuilder;

use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::i2c::AtomicDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_hal_bus::util::AtomicCell;
use mipidsi::interface::SpiInterface;
use mipidsi::models::ST7789;
use mipidsi::options::ColorInversion;
use mipidsi::options::Orientation;
use opt300x::Opt300x;
use bmi160::Bmi160;
use panic_halt as _;

use embedded_graphics::{prelude::*, mono_font::ascii::FONT_6X10, pixelcolor::Rgb565, text::Text};
use mipidsi::Builder;

use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::gpio::PullUp;
use tm4c123x_hal::i2c::I2C;
use tm4c123x_hal::pac::I2C1;
use tm4c123x_hal::pac::SSI0;
use tm4c123x_hal::sysctl::SysctlExt;
use tm4c123x_hal::{self as hal, prelude::*, pac, gpio::{Floating, AF3}};
use cortex_m_rt::entry;

use core::{fmt::Write};

#[entry]
fn main() -> ! {
    let cortex_p = if let Some(cortex_p) = pac::CorePeripherals::take() {
        cortex_p
    } else {
        panic!("Failed to take Cortex-M peripherals");
    };

    let p = pac::Peripherals::take().unwrap();
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    let mut delay_source = hal::delay::Delay::new(cortex_p.SYST, &clocks);

    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let porte = p.GPIO_PORTE.split(&sc.power_control);
    let mut portf = p.GPIO_PORTF.split(&sc.power_control);
   
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

    let spi = hal::spi::Spi::<SSI0, _>::spi0(
        p.SSI0,
        (
            porta.pa2.into_af_push_pull::<hal::gpio::AF2>(&mut porta.control),
            porta.pa4.into_af_open_drain::<hal::gpio::AF2, Floating>(&mut porta.control),
            porta.pa5.into_af_push_pull::<hal::gpio::AF2>(&mut porta.control),
        ),
        hal::spi::MODE_3,
        20.mhz(),
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

    let mut tft_rst = porte.pe0.into_push_pull_output();
    tft_rst.set_high().unwrap();
    let tft_dc = portf.pf3.into_push_pull_output();
    let tft_cs = portf.pf4.into_push_pull_output();
    let mut sw_2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();

    let mut spi_buffer = [0u8; 256];
    let spi_device = ExclusiveDevice::new_no_delay(spi, tft_cs).unwrap();
    let di = SpiInterface::new(spi_device, tft_dc, &mut spi_buffer);
    // let mut display = Builder::new(models::ST7789, di)
    //     // .with_invert_colors(mipidsi::ColorInversion::Inverted)
    //     // .with_orientation(mipidsi::Orientation::PortraitInverted(false))
    //     .invert_colors(mipidsi::options::ColorInversion::Inverted)
    //     .orientation(mipidsi::options::Orientation::new().flip_horizontal())
    //     .init(&mut delay_source)

    //     .unwrap();
    let mut display = Builder::new(ST7789, di)
        .invert_colors(ColorInversion::Inverted)
        .orientation(Orientation::new())
        .init(&mut delay_source)
        .unwrap();

    let y_margin = (50, 270);
    let mut cursor = 0;
    display.clear(Rgb565::BLACK).unwrap();
    let mut str_buffer: heapless::String<64> = heapless::String::new();

    loop {
        let lux = opt3001.read_lux().unwrap();
        let accel_x = bmi160.data(selector).unwrap().accel.unwrap().x;

        write!(str_buffer, "OPT3001: {} lux   ", lux).unwrap();
        Text::new(&str_buffer, Point::new(20, y_margin.0 + cursor), style).draw(&mut display).unwrap();
        str_buffer.clear();

        write!(str_buffer, "BMI160: {}           ", accel_x).unwrap();
        Text::new(&str_buffer, Point::new(20, y_margin.0 + cursor + 20), style).draw(&mut display).unwrap();
        str_buffer.clear();
        cursor = cursor + 40;

        if cursor > (y_margin.1 - y_margin.0) {
            cursor = 0;
        }

        while sw_2.is_high().unwrap() {
            cortex_m::asm::nop();
        }
        while sw_2.is_low().unwrap() {
            cortex_m::asm::nop();
        }

    }
    // loop {}
}
