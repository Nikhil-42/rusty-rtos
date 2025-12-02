#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::{mem::MaybeUninit, sync::atomic::AtomicBool, time};

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

use pca9555::Pca9555;
use tm4c123x_hal::{
    self as hal,
    adc::{AdcSingle, TriggerSource},
    gpio::{
        AF1, AF2, AF3, AF5, AlternateFunction, Analog, Floating, Input, OpenDrain, Output, PullUp, PushPull, gpioa::{PA0, PA1, PA2, PA4, PA5}, gpiob::{PB2, PB3, PB4, PB6, PB7}, gpioc::{PC4, PC5, PC6, PC7}, gpioe::{PE0, PE2}, gpiof::{PF1, PF3, PF4}
    },
    i2c::I2C,
    pac::{self, ADC0, ADC1, I2C0, I2C1, SSI0, SSI2, TIMER0, TIMER1, UART0, UART3, UART4},
    prelude::*,
    serial::Serial,
    spi::Spi,
};

use cortex_m_rt::entry;
use eh0::{
    serial::{Read as _, Write as _},
};
use embedded_hal::{digital::OutputPin, i2c::I2c, spi::SpiDevice};

static mut JOYSTICK_Y_ADC_S: MaybeUninit<AdcSingle<ADC1, PE2<Analog<Input<Floating>>>>> =
    MaybeUninit::uninit();
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
static mut DAC_TIMER_S: MaybeUninit<
    hal::timer::Timer<TIMER0>,
> = MaybeUninit::uninit();
static mut DAC_SPI_S: MaybeUninit<
    ExclusiveDevice<
        Spi<
            SSI2,
            (
                PB4<AlternateFunction<AF2, PushPull>>,
                PB6<AlternateFunction<AF2, OpenDrain<Floating>>>,
                PB7<AlternateFunction<AF2, PushPull>>,
            ),
        >,
        PF1<Output<PushPull>>,
        NoDelay,
    >,
> = MaybeUninit::uninit();

static MIC_ADC_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();
static JOYSTICK_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();
static UART3_TX_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();
static UART0_TX_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();

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
static I2C0_MUTEX: G8torMutex<
    I2C<
        I2C0,
        (
            PB2<AlternateFunction<AF3, PushPull>>,
            PB3<AlternateFunction<AF3, OpenDrain<PullUp>>>,
        ),
    >,
> = G8torMutex::empty();
static TONE_SELECT: AtomicBool = AtomicBool::new(false);

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

extern "C" fn mic_adc() {
    let mic_fifo_handle = &*MIC_ADC_FIFO;
    let adc = unsafe { &*pac::ADC0::ptr() };

    // Clear interrupt flag
    adc.isc.write(|w| w.in0().set_bit());

    // Dump the hardware FIFO into software
    while adc.ssfstat0.read().empty().bit_is_clear() {
        let result = adc.ssfifo0.read().data().bits();
        rtos::write_fifo(mic_fifo_handle, result as u32);
    }
}


const A4: [u16; 100] = [
    0x800, 0x880, 0x900, 0x97f, 0x9fd, 0xa78, 0xaf1, 0xb67, 0xbda, 0xc49,
    0xcb3, 0xd19, 0xd79, 0xdd4, 0xe29, 0xe78, 0xec0, 0xf02, 0xf3c, 0xf6f,
    0xf9b, 0xfbf, 0xfdb, 0xfef, 0xffb, 0xfff, 0xffb, 0xfef, 0xfdb, 0xfbf,
    0xf9b, 0xf6f, 0xf3c, 0xf02, 0xec0, 0xe78, 0xe29, 0xdd4, 0xd79, 0xd19,
    0xcb3, 0xc49, 0xbda, 0xb67, 0xaf1, 0xa78, 0x9fd, 0x97f, 0x900, 0x880,
    0x800, 0x77f, 0x6ff, 0x680, 0x602, 0x587, 0x50e, 0x498, 0x425, 0x3b6,
    0x34c, 0x2e6, 0x286, 0x22b, 0x1d6, 0x187, 0x13f, 0xfd, 0xc3, 0x90,
    0x64, 0x40, 0x24, 0x10, 0x4, 0x0, 0x4, 0x10, 0x24, 0x40,
    0x64, 0x90, 0xc3, 0xfd, 0x13f, 0x187, 0x1d6, 0x22b, 0x286, 0x2e6,
    0x34c, 0x3b6, 0x425, 0x498, 0x50e, 0x587, 0x602, 0x680, 0x6ff, 0x77f,  
];

const A8: [u16; 50] = [
    0x800, 0x900, 0x9fd, 0xaf1, 0xbda, 0xcb3, 0xd79, 0xe29, 0xec0, 0xf3c,
    0xf9b, 0xfdb, 0xffb, 0xffb, 0xfdb, 0xf9b, 0xf3c, 0xec0, 0xe29, 0xd79,
    0xcb3, 0xbda, 0xaf1, 0x9fd, 0x900, 0x800, 0x6ff, 0x602, 0x50e, 0x425,
    0x34c, 0x286, 0x1d6, 0x13f, 0xc3, 0x64, 0x24, 0x4, 0x4, 0x24,
    0x64, 0xc3, 0x13f, 0x1d6, 0x286, 0x34c, 0x425, 0x50e, 0x602, 0x6ff
];

extern "C" fn speaker_dac() {
    // Clear interrupt flag
    let dac_timer = unsafe { &mut *DAC_TIMER_S.as_mut_ptr() };
    dac_timer.clear_interrupt(hal::timer::Event::TimeOut);
    
    // Write samples to DAC via SPI
    static mut SAMPLE_INDEX: usize = 0;

    let dac_spi = unsafe { DAC_SPI_S.assume_init_mut() };
    let tone = TONE_SELECT.load(core::sync::atomic::Ordering::Relaxed);

    // SAFETY: this function is only called from a single interrupt context, so the mutable static is safe to use.
    let sample = unsafe { 
        let sample_idx = SAMPLE_INDEX;
        SAMPLE_INDEX += 1;
        let sample = if tone { 
            SAMPLE_INDEX %= A8.len();
            A8[sample_idx]
        } else {
            SAMPLE_INDEX %= A4.len();
            A4[sample_idx]
        };

        sample
    };

    const COMMAND_0: u8 = 0b00000_00_0;
    const COMMAND_1: u8 = 0b00001_00_0;
    let tx_buf = [COMMAND_0, (sample >> 8) as u8, (sample & 0xFF) as u8, COMMAND_1, (sample >> 8) as u8, (sample & 0xFF) as u8];
    dac_spi.write(&tx_buf).unwrap();
}

extern "C" fn initialize(rtos: G8torThreadHandle) -> ! {
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

// extern "C" fn debug_tx(_rtos: G8torThreadHandle) -> ! {
//     let mic_fifo_handle = &*MIC_ADC_FIFO;
//     let mut uart_handle = &*UART0_MUT;

//     loop {
//         let sample = rtos::read_fifo(mic_fifo_handle) as u16;

//         // Transmit sample over UART0
//         let bytes = sample.to_le_bytes();
//         for &b in bytes.iter() {
//             uart_handle.write(b).unwrap();
//         }
//     }
// }

extern "C" fn read_joystick_adc() {
    // Read the joystick ADC values and push to FIFO every 100ms
    unsafe {
        // let x_adc = JOYSTICK_X_ADC_S.assume_init_mut();
        let y_adc = JOYSTICK_Y_ADC_S.assume_init_mut();
        // let x_val = x_adc.read();
        let y_val = y_adc.read();
        let joystick_fifo_handle = &*JOYSTICK_FIFO;
        let packed_val: u32 = ((y_val as u32) << 16) | (0 as u32);
        rtos::write_fifo(joystick_fifo_handle, packed_val);
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
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let mut portc = p.GPIO_PORTC.split(&sc.power_control);
    let porte = p.GPIO_PORTE.split(&sc.power_control);
    let portf = p.GPIO_PORTF.split(&sc.power_control);

    // Activate I2C1
    let i2c1 = hal::i2c::I2C::<I2C1, _>::new(
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
    
    // Activate the Joystick Y ADCs
    let pe2 = porte.pe2.into_floating_input().into_analog();
    let joystick_y_adc = AdcSingle::adc1(p.ADC1, pe2, &sc.power_control);

    // Activate Timer to trigger audio DAC/ADC
    let mut dac_timer = hal::timer::Timer::timer0(p.TIMER0, 44.khz(), &clocks, &sc.power_control);
    dac_timer.listen(hal::timer::Event::TimeOut);

    let timer0_raw = unsafe { &*pac::TIMER0::ptr() };
    timer0_raw.ctl.modify(|_, w| w.taen().clear_bit()); // Stop timer to configure
    timer0_raw.ctl.modify(|_, w| w.taote().set_bit()); // Enable ADC trigger on timeout
    timer0_raw.ctl.modify(|_, w| w.taen().set_bit()); // Start timer

    // Activate the Microphone ADC
    let pe1 = porte.pe1.into_floating_input().into_analog();

    let mut adc_mic = AdcSingle::adc0(p.ADC0, pe1, &sc.power_control);
    adc_mic.set_trigger(hal::adc::TriggerSource::Timer);
    adc_mic.enable_interrupt();

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
    let spi0 = hal::spi::Spi::spi0(
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

    let spi_device = ExclusiveDevice::new(spi0, tft_cs, NoDelay).unwrap();
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
        .orientation(
            mipidsi::options::Orientation::new()
                .flip_horizontal()
                .flip_vertical(),
        )
        .reset_pin(tft_rst)
        .init(&mut delay_source)
        .unwrap();
    // 240 x 320 resolution

    core_p.SYST = delay_source.free();
    display.clear(Rgb565::BLACK).unwrap();

    // Activate DAC SPI 
    let spi2 = hal::spi::Spi::spi2(
        p.SSI2,
        (
            portb.pb4.into_af_push_pull::<hal::gpio::AF2>(&mut portb.control),
            portb.pb6.into_af_open_drain::<hal::gpio::AF2, Floating>(&mut portb.control),
            portb.pb7.into_af_push_pull::<hal::gpio::AF2>(&mut portb.control),
        ),
        hal::spi::MODE_3,
        20.mhz(),
        &clocks,
        &sc.power_control,
    );

    let dac_cs = portf.pf1.into_push_pull_output();
    let dac_spi = ExclusiveDevice::new(spi2, dac_cs, NoDelay).unwrap();

    let inst = unsafe { rtos::G8torRtos::new(core_p) };

    let uart0_handle = inst
        .init_mutex(&UART0_MUTEX)
        .expect("We haven't run out of atomics");

    let i2c0_handle = inst
        .init_mutex(&I2C0_MUTEX)
        .expect("We haven't run out of atomics");

    inst.add_thread(&byte_str("init"), 0, initialize)
        .expect("Failed to add init thread");

    inst.add_periodic(100, 0, read_joystick_adc)
        .expect("Periodic TCB list has space");

    let mic_adc_fifo = inst.init_fifo().expect("We haven't run out of atomics");
    let joystick_fifo = inst.init_fifo().expect("We haven't run out of atomics");

    inst.add_event(pac::Interrupt::ADC0SS0, 1, mic_adc)
        .expect("Failed to add ADC0SS0 event");
    // inst.add_event(pac::Interrupt::ADC1SS0, 2, joystick_y_adc)
    //     .expect("Failed to add ADC1SS0 event");
    inst.add_event(pac::Interrupt::TIMER0A, 1, speaker_dac)
        .expect("Failed to add TIMER1A event");

    unsafe {
        SCREEN_S.as_mut_ptr().write(display);
        DAC_TIMER_S.as_mut_ptr().write(dac_timer);
        DAC_SPI_S.as_mut_ptr().write(dac_spi);
        JOYSTICK_Y_ADC_S.as_mut_ptr().write(joystick_y_adc);

        UART0_MUTEX.init(uart0);
        UART0_MUT.set(uart0_handle);

        I2C0_MUTEX.init(i2c0);
        I2C0_MUT.set(i2c0_handle);

        MIC_ADC_FIFO.set(mic_adc_fifo);
        JOYSTICK_FIFO.set(joystick_fifo);

        inst.launch()
    }
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