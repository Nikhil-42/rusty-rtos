#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use cortex_m_rt::entry;
use embedded_hal::spi::SpiDevice as _;
use panic_halt as _;
use pca9555::Pca9555;
use tm4c123x_hal::gpio::gpiob::PB2;
use tm4c123x_hal::gpio::gpiob::PB3;
use tm4c123x_hal::gpio::gpiob::PB4;
use tm4c123x_hal::gpio::gpiob::PB6;
use tm4c123x_hal::gpio::gpiob::PB7;
use tm4c123x_hal::gpio::gpioe::PE1;
use tm4c123x_hal::gpio::gpiof::PF1;
use tm4c123x_hal::pac::I2C0;
use tm4c123x_hal::pac::SSI2;
use tm4c123x_hal::pac::TIMER0;
use tm4c123x_hal::pac::i2c0;
use tm4c123x_hal::timer::Timer;

use core::convert::{TryFrom, TryInto};
use core::mem::MaybeUninit;
use core::sync::atomic::AtomicU8;
use core::sync::atomic::AtomicU16;
use core::sync::atomic::Ordering;

use eel4745c::rtos::{
    self, G8torMutex, G8torMutexHandle, G8torRtos, G8torSemaphoreHandle,
    G8torThreadHandle,
};
use eel4745c::{byte_str, SyncUnsafeOnceCell};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use mipidsi::{interface::SpiInterface, Builder, Display};

use embedded_hal::{digital::OutputPin, i2c::I2c};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

use tm4c123x_hal::{
    self as hal,
    adc::AdcSingle,
    gpio::{
        gpioa::{PA0, PA1, PA2, PA4, PA5, PA6, PA7},
        gpioe::{PE0, PE2},
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

// Sine Wave Samples for 1000 Hz Tone at 11 kHz Sample Rate
const SINE_WAVE: [u16; 42] = [
    0x800, 0x931, 0xa5b, 0xb78, 0xc81, 0xd70, 0xe40, 0xeed, 0xf71, 0xfcc, 0xff9, 0xff9,
    0xfcc, 0xf71, 0xeed, 0xe40, 0xd70, 0xc81, 0xb78, 0xa5b, 0x931, 0x800, 0x6ce, 0x5a4,
    0x487, 0x37e, 0x28f, 0x1bf, 0x112, 0x8e, 0x33, 0x6, 0x6, 0x33, 0x8e, 0x112,
    0x1bf, 0x28f, 0x37e, 0x487, 0x5a4, 0x6ce, 
];

enum PlaybackMode {
    Off = 0,
    Tone1k = 1,
    Tone2k = 2,
    Tone3k = 3,
    Loopback = 4,
}

impl TryFrom<u8> for PlaybackMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(PlaybackMode::Off),
            1 => Ok(PlaybackMode::Tone1k),
            2 => Ok(PlaybackMode::Tone2k),
            3 => Ok(PlaybackMode::Tone3k),
            4 => Ok(PlaybackMode::Loopback),
            _ => Err(()),
        }
    }
}

extern "C" fn mic_adc_isr() {
    let adc = unsafe { &*pac::ADC0::ptr() };

    // Clear interrupt flag
    adc.isc.write(|w| w.in0().set_bit());

    // Dump the hardware FIFO into software
    let mut result = 0x000;
    while adc.ssfstat0.read().empty().bit_is_clear() {
        result = adc.ssfifo0.read().data().bits();
    }
    LAST_SAMPLE.store(result, core::sync::atomic::Ordering::Relaxed);
}

extern "C" fn speaker_timer_isr() {
    // Clear interrupt flag
    let dac_timer = unsafe { &mut *DAC_TIMER_S.as_mut_ptr() };
    dac_timer.clear_interrupt(hal::timer::Event::TimeOut);
    
    let mode: Result<PlaybackMode, ()> = PLAYBACK_MODE.load(Ordering::Relaxed).try_into();
    let sample = match mode {
        Ok(PlaybackMode::Off) => 0,
        Ok(PlaybackMode::Loopback) => {
            LAST_SAMPLE.load(Ordering::Relaxed)
        },
        Ok(tone) => {
            // Frequency is controlled by skipping samples
            static mut SAMPLE_INDEX: usize = 0;

            let inc = tone as usize;
            let tap = SINE_WAVE[unsafe {
                let index = SAMPLE_INDEX;
                SAMPLE_INDEX = (SAMPLE_INDEX + inc) % SINE_WAVE.len();
                index
            }];
            (tap as f32 * (f32::from(VOLUME.load(Ordering::Relaxed)) / (0xFFF as f32))) as u16
        },
        _ => 0,
    };

    let dac_spi = unsafe { DAC_SPI_S.assume_init_mut() };
    const COMMAND_0: u8 = 0b00000_00_0;
    const COMMAND_1: u8 = 0b00001_00_0;
    let tx_buf = [COMMAND_0, (sample >> 8) as u8, (sample & 0xFF) as u8, COMMAND_1, (sample >> 8) as u8, (sample & 0xFF) as u8];
    dac_spi.write(&tx_buf).unwrap();
}

#[entry]
fn main() -> ! {
    initialize(
        |_| {},
        Some(|| {
            PLAYBACK_MODE.store(PlaybackMode::Tone1k as u8, Ordering::Relaxed);
        }),
        Some(|| {
            PLAYBACK_MODE.store(PlaybackMode::Tone2k as u8, Ordering::Relaxed);
        }),
        Some(|| {
            PLAYBACK_MODE.store(PlaybackMode::Tone3k as u8, Ordering::Relaxed);
        }),
        Some(|| {
            PLAYBACK_MODE.store(PlaybackMode::Off as u8, Ordering::Relaxed);
        }),
        Some(|| {
            PLAYBACK_MODE.store(PlaybackMode::Loopback as u8, Ordering::Relaxed);
        })
    )
}

// Standard Resources
// Single Consumer Resources
static mut MICROPHONE_ADC_S: MaybeUninit<AdcSingle<ADC0, PE1<Analog<Input<Floating>>>>> =
    MaybeUninit::uninit();
static mut JOYSTICK_Y_ADC_S: MaybeUninit<AdcSingle<ADC1, PE2<Analog<Input<Floating>>>>> =
    MaybeUninit::uninit();
static mut DAC_TIMER_S: MaybeUninit<Timer<TIMER0>> = MaybeUninit::uninit();
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

// Shared Resources
static VOLUME: AtomicU16 = AtomicU16::new(0);
static LAST_SAMPLE: AtomicU16 = AtomicU16::new(0);
static PLAYBACK_MODE: AtomicU8 = AtomicU8::new(0);
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
static I2C1_MUTEX: G8torMutex<
    I2C<
        I2C1,
        (
            PA6<AlternateFunction<AF3, PushPull>>,
            PA7<AlternateFunction<AF3, OpenDrain<PullUp>>>,
        ),
    >,
> = G8torMutex::empty();
static UART0_MUTEX: G8torMutex<
    Serial<
        UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
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
static I2C1_MUT: SyncUnsafeOnceCell<
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

// FIFOs

// Semaphore Handles
static JOYSTICK_PUSH_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();
static BUTTONS_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();

static ON_SW1: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();
static ON_SW2: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();
static ON_SW3: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();
static ON_SW4: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();
static ON_JOYSTICK: SyncUnsafeOnceCell<Option<fn()>> = SyncUnsafeOnceCell::new();

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

fn initialize(
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
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let portd = p.GPIO_PORTD.split(&sc.power_control);
    let porte = p.GPIO_PORTE.split(&sc.power_control);
    let portf = p.GPIO_PORTF.split(&sc.power_control);

    // Enable buttons interrupt
    let mut pe4 = porte.pe4.into_pull_up_input();
    pe4.set_interrupt_mode(hal::gpio::InterruptMode::EdgeFalling);

    // Initialize Joystick ADC
    let pe2 = porte.pe2.into_floating_input().into_analog();
    let adc1 = AdcSingle::adc1(p.ADC1, pe2, &sc.power_control);

    // Enable Joystick Interrupt
    let mut pd2 = portd.pd2.into_pull_up_input();
    pd2.set_interrupt_mode(hal::gpio::InterruptMode::EdgeFalling);

    // Activate Timer to trigger audio DAC/ADC
    let mut dac_timer = hal::timer::Timer::timer0(p.TIMER0, 42.khz(), &clocks, &sc.power_control);
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

    // Activate UART
    let uart0 = Serial::uart0(
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

    // Activate I2C1 module
    let i2c1: I2C<
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

    // Activate SPI0 module
    let spi0 = Spi::<SSI0, _>::spi0(
        p.SSI0,
        (
            porta
                .pa2
                .into_af_push_pull::<AF2>(&mut porta.control),
            porta
                .pa4
                .into_af_open_drain::<AF2, Floating>(&mut porta.control),
            porta
                .pa5
                .into_af_push_pull::<AF2>(&mut porta.control),
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

    // Activate SPI2 module for DAC
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

    // Initialize RTOS
    let inst = unsafe { rtos::G8torRtos::new(core_p) };

    let screen_mut = inst
        .init_mutex(&SCREEN_MUTEX)
        .expect("We haven't run out of atomics");
    let i2c0_mut = inst
        .init_mutex(&I2C0_MUTEX)
        .expect("We haven't run out of atomics"); 
    let i2c1_mut = inst
        .init_mutex(&I2C1_MUTEX)
        .expect("We haven't run out of atomics");
    let uart0_mut = inst
        .init_mutex(&UART0_MUTEX)
        .expect("We haven't run out of atomics");

    let joystick_push_sem = inst
        .init_semaphore(0)
        .expect("We haven't run out of atomics");
    let buttons_sem = inst
        .init_semaphore(0)
        .expect("We haven't run out of atomics");

    inst.add_thread(&byte_str("configure_audio"), 1, configure_audio)
        .expect("TCB list has space");
    inst.add_thread(&byte_str("read_buttons"), 1, read_buttons)
        .expect("TCB list has space");
    inst.add_thread(&byte_str("read_joystick"), 1, read_joystick)
        .expect("TCB list has space");

    inst.add_periodic(100, 0, read_joystick_adc)
        .expect("Periodic TCB list has space");

    inst.add_event(pac::interrupt::GPIOD, 5, joystick_click_isr)
        .expect("Inputs are correct");
    inst.add_event(pac::interrupt::GPIOE, 5, button_isr)
        .expect("Inputs are correct");
    // Add ISR for Microphone ADC
    inst.add_event(pac::interrupt::ADC0SS0, 3, mic_adc_isr)
        .expect("Inputs are correct");
    // Add ISR for Speaker Timer
    inst.add_event(pac::interrupt::TIMER0A, 2, speaker_timer_isr)
        .expect("Inputs are correct");

    unsafe {
        MICROPHONE_ADC_S.write(adc0);
        JOYSTICK_Y_ADC_S.write(adc1);
        DAC_TIMER_S.write(dac_timer);
        DAC_SPI_S.write(dac_spi);

        SCREEN_MUTEX.init(display);
        I2C1_MUTEX.init(i2c1);
        I2C0_MUTEX.init(i2c0);
        UART0_MUTEX.init(uart0);

        SCREEN_MUT.set(screen_mut);
        I2C1_MUT.set(i2c1_mut);
        I2C0_MUT.set(i2c0_mut);
        UART0_MUT.set(uart0_mut);

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

extern "C" fn read_joystick_adc() {
    // Read the joystick y ADC value and update volume every 100ms
    let y_adc = unsafe { JOYSTICK_Y_ADC_S.assume_init_mut() };
    let y_val = y_adc.read();
    VOLUME.store(y_val, Ordering::Relaxed);
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
    let mut i2c = *I2C1_MUT;
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
