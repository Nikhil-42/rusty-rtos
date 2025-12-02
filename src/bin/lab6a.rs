#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use cortex_m_rt::entry;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use micromath::F32Ext;
use panic_halt as _;
use pca9555::Pca9555;
use tm4c123x_hal::gpio::gpiob::{PB2, PB3};
use tm4c123x_hal::gpio::gpioe::PE1;
use tm4c123x_hal::pac::{I2C0, TIMER0};
use tm4c123x_hal::timer::Timer;

use core::mem::MaybeUninit;

use eel4745c::rtos::{
    self, G8torFifoHandle, G8torMutex, G8torMutexHandle, 
    G8torThreadHandle,
};
use eel4745c::{byte_str, SyncUnsafeOnceCell};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use mipidsi::{interface::SpiInterface, Builder, Display};

use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

use tm4c123x_hal::{
    self as hal,
    prelude::*,
    adc::AdcSingle,
    gpio::{
        gpioa::{PA2, PA4, PA5},
        gpioe::PE0,
        gpiof::{PF3, PF4},
        AlternateFunction, Analog, Floating, Input, OpenDrain, Output, PullUp, PushPull, AF2,
        AF3,
    },
    i2c::I2C,
    pac::{self, ADC0, SSI0},
    spi::Spi,
};

const SAMPLING_FREQUENCY: u32 = 42_000;
const COEFFICIENTS: [(f32, (f32, f32, f32, f32)); 2] = [
    (
        1.99566878548,
        (
            0.245790784309,
            0.969322903035,
            -0.309016994375,
            -0.951056516295,
        ),
    ), //  440 A4
    (
        1.98269390132,
        (
            -0.879173780697,
            0.476501273172,
            0.809016994375,
            -0.587785252292,
        ),
    ), // 523.251 C5
];

extern "C" fn mic_adc_isr() {
    const N: u32 = SAMPLING_FREQUENCY / 50;

    let adc = unsafe { &*pac::ADC0::ptr() };

    // Clear interrupt flag
    adc.isc.write(|w| w.in0().set_bit());

    // Dump the hardware FIFO into software
    while adc.ssfstat0.read().empty().bit_is_clear() {
        static mut SAMPLE_INDEX: usize = 0;
        static mut GOERTZEL_W: [(f32, f32); COEFFICIENTS.len()] = [(0.0, 0.0); COEFFICIENTS.len()];

        let result = adc.ssfifo0.read().data().bits();

        for i in 0..COEFFICIENTS.len() {
            let w = unsafe { &mut GOERTZEL_W[i] };
            let (c, _) = COEFFICIENTS[i];

            // Implementation of Geortzel's algorithm
            let new_w = result as f32 + c * w.0 - w.1;
            w.1 = w.0;
            w.0 = new_w;
        }

        unsafe {
            SAMPLE_INDEX = (SAMPLE_INDEX + 1) % N as usize;
            if SAMPLE_INDEX == 0 {
                // Dump measured amplitude to FIFO
                let sqr_mag_fifo = &*SQR_MAG_FIFO;

                rtos::write_fifo(sqr_mag_fifo, 0xFFFFFFFF);
                for i in 0..COEFFICIENTS.len() {
                    let w = &mut GOERTZEL_W[i];
                    let (_, (a, b, c, d)) = COEFFICIENTS[i];

                    let real = a * w.0 + c * w.1;
                    let imag = b * w.0 - d * w.1;
                    let magnitude = (real * real + imag * imag).sqrt();
                    let magnitude = f32::max((magnitude - 500_000f32) / 10_000.0, 0.0);

                    // Reset Goertzel state
                    w.0 = 0.0;
                    w.1 = 0.0;

                    rtos::write_fifo(sqr_mag_fifo, magnitude as u32);
                }
            }
        }
    }
}

extern "C" fn display_bars(_rtos: G8torThreadHandle) -> ! {
    let screen_mut = &*SCREEN_MUT;
    let sqr_mag_fifo = &*SQR_MAG_FIFO;

    let mut magnitudes: [u32; COEFFICIENTS.len()] = [0; COEFFICIENTS.len()];
    const BAR_WIDTH: u32 = 200 / (2 * COEFFICIENTS.len() as u32);
    loop {
        // Clear old magnitude bars
        let display = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        for (i, magn) in magnitudes.iter().enumerate() {
            let x = 20 + (BAR_WIDTH as i32) / 2 + (i as i32) * (BAR_WIDTH as i32) * 2;
            let y = 240;
            let width = u32::min(BAR_WIDTH, 10);
            let height = u32::min(*magn, 200);

            let rect = Rectangle::new(Point::new(x, y - (height as i32)), Size::new(width as u32, height));
            rect.into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                .draw(display)
                .unwrap();
        }
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(display));

        while rtos::read_fifo(sqr_mag_fifo) != 0xFFFFFFFF {}
        for magn in magnitudes.iter_mut() {
            *magn = rtos::read_fifo(sqr_mag_fifo);
        }

        // Clear old magnitude bars
        let display = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        for (i, magn) in magnitudes.iter().enumerate() {
            let x = 20 + (BAR_WIDTH as i32) / 2 + (i as i32) * (BAR_WIDTH as i32) * 2;
            let y = 240;
            let width = u32::min(BAR_WIDTH, 10);
            let height = u32::min(*magn, 200);

            let rect = Rectangle::new(Point::new(x, y - (height as i32)), Size::new(width as u32, height));
            rect.into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
                .draw(display)
                .unwrap();
        }
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(display));

        rtos::sleep_ms(200);
    }
}

// Standard Resources
// Single Consumer Resources
static mut MICROPHONE_ADC_S: MaybeUninit<AdcSingle<ADC0, PE1<Analog<Input<Floating>>>>> =
    MaybeUninit::uninit();
static mut SAMPLE_TIMER_S: MaybeUninit<Timer<TIMER0>> = MaybeUninit::uninit();

// Shared Resources
static SCREEN_MUTEX: G8torMutex<
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
static I2C0_MUTEX: G8torMutex<
    I2C<
        I2C0,
        (
            PB2<AlternateFunction<AF3, PushPull>>,
            PB3<AlternateFunction<AF3, OpenDrain<PullUp>>>,
        ),
    >,
> = G8torMutex::empty();

// Mutex Handles
static SCREEN_MUT: SyncUnsafeOnceCell<
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
static I2C0_MUT: SyncUnsafeOnceCell<
    G8torMutexHandle<
        I2C<
            I2C0,
            (
                PB2<AlternateFunction<AF3, PushPull>>,
                PB3<AlternateFunction<AF3, OpenDrain<PullUp>>>,
            ),
        >,
    >,
> = SyncUnsafeOnceCell::new();

// FIFOs
static SQR_MAG_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();

// Semaphore Handles

extern "C" fn configure_audio(rtos: G8torThreadHandle) -> ! {
    let i2c0 = &*I2C0_MUT;

    // Activate the Microphone Amplifier
    let audio_gpio: Pca9555<_, 0x22> = Pca9555::new(*i2c0);
    let audio_gpio_port = audio_gpio.split();

    let mut mic_amp_en = audio_gpio_port.pin0.into_output().unwrap();
    let mut speaker_amp_en = audio_gpio_port.pin4.into_output().unwrap();
    let mut dac_lat_pin = audio_gpio_port.pin1.into_output().unwrap();

    mic_amp_en.set_high().unwrap();
    speaker_amp_en.set_high().unwrap();
    dac_lat_pin.set_low().unwrap();

    rtos.kill();
}

#[entry]
fn main() -> ! {
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
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let porte = p.GPIO_PORTE.split(&sc.power_control);
    let portf = p.GPIO_PORTF.split(&sc.power_control);

    // Activate Timer to trigger audio DAC/ADC
    let mut dac_timer = hal::timer::Timer::timer0(
        p.TIMER0,
        SAMPLING_FREQUENCY.hz(),
        &clocks,
        &sc.power_control,
    );
    dac_timer.listen(hal::timer::Event::TimeOut);

    let timer0_raw = unsafe { &*pac::TIMER0::ptr() };
    timer0_raw.ctl.modify(|_, w| w.taen().clear_bit()); // Stop timer to configure
    timer0_raw.ctl.modify(|_, w| w.taote().set_bit()); // Enable ADC trigger on timeout
    timer0_raw.ctl.modify(|_, w| w.taen().set_bit()); // Start timer

    // Activate the Microphone ADC
    let pe1 = porte.pe1.into_floating_input().into_analog();

    let mut adc0 = AdcSingle::adc0(p.ADC0, pe1, &sc.power_control);
    adc0.set_trigger(hal::adc::TriggerSource::Timer);
    adc0.enable_interrupt();

    // Activate I2C0
    let i2c0 = hal::i2c::I2C::<I2C0, _>::new(
        p.I2C0,
        (
            portb.pb2.into_af_push_pull::<AF3>(&mut portb.control),
            portb
                .pb3
                .into_af_open_drain::<AF3, PullUp>(&mut portb.control),
        ),
        100.khz(),
        &clocks,
        &sc.power_control,
    );

    // Activate SPI0 module
    let spi0 = Spi::<SSI0, _>::spi0(
        p.SSI0,
        (
            porta.pa2.into_af_push_pull::<AF2>(&mut porta.control),
            porta
                .pa4
                .into_af_open_drain::<AF2, Floating>(&mut porta.control),
            porta.pa5.into_af_push_pull::<AF2>(&mut porta.control),
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
    let spi_device = ExclusiveDevice::new(spi0, tft_cs, NoDelay).unwrap();
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
    let i2c0_mut = inst
        .init_mutex(&I2C0_MUTEX)
        .expect("We haven't run out of atomics");

    let sqr_mag_fifo = inst.init_fifo().expect("We haven't run out of FIFOs");

    inst.add_thread(&byte_str("configure_audio"), 1, configure_audio)
        .expect("TCB list has space");
    inst.add_thread(&byte_str("display_bars"), 2, display_bars)
        .expect("TCB list has space");

    // Add ISR for Microphone ADC
    inst.add_event(pac::interrupt::ADC0SS0, 3, mic_adc_isr)
        .expect("Inputs are correct");

    unsafe {
        MICROPHONE_ADC_S.write(adc0);
        SAMPLE_TIMER_S.write(dac_timer);

        SCREEN_MUTEX.init(display);
        I2C0_MUTEX.init(i2c0);

        SCREEN_MUT.set(screen_mut);
        I2C0_MUT.set(i2c0_mut);

        SQR_MAG_FIFO.set(sqr_mag_fifo);
        inst.launch()
    }
}
