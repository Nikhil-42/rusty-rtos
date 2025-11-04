#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::fmt::Write as _;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};
use eel4745c::{byte_str, SyncUnsafeOnceCell};

use eel4745c::graphics::Cube;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_hal::{digital::OutputPin, i2c::I2c};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use mipidsi::Display;
use mipidsi::{interface::SpiInterface, Builder};

use eel4745c::rtos::{
    self, G8torFifoHandle, G8torMutex, G8torMutexHandle, G8torSemaphoreHandle, G8torThreadHandle,
};
use panic_halt as _;

use rand::{Rng, SeedableRng};
use tm4c123x_hal::adc::AdcSingle;
use tm4c123x_hal::gpio::gpioa::{PA6, PA7};
use tm4c123x_hal::gpio::gpioe::{PE2, PE3};
use tm4c123x_hal::gpio::{
    gpioa::{PA2, PA4, PA5},
    gpioe::PE0,
    gpiof::{PF3, PF4},
    Floating, OpenDrain, Output, AF2,
};
use tm4c123x_hal::gpio::{Analog, Input, PullUp, AF3};
use tm4c123x_hal::i2c::I2C;
use tm4c123x_hal::pac::{ADC0, ADC1, I2C1};
use tm4c123x_hal::spi::Spi;
use tm4c123x_hal::{
    self as hal,
    gpio::{
        gpioa::{PA0, PA1},
        AlternateFunction, PushPull, AF1,
    },
    pac::{self, SSI0, UART0},
    prelude::*,
    serial::Serial,
};

use cortex_m_rt::entry;

static JOYSTICK_FLAG: AtomicBool = AtomicBool::new(false);

static mut UART0_S: MaybeUninit<
    Serial<
        UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
> = MaybeUninit::uninit();
static mut JOYSTICK_X_ADC: MaybeUninit<AdcSingle<ADC0, PE3<Analog<Input<Floating>>>>> =
    MaybeUninit::uninit();
static mut JOYSTICK_Y_ADC: MaybeUninit<AdcSingle<ADC1, PE2<Analog<Input<Floating>>>>> =
    MaybeUninit::uninit();
static KILL_CUBE: AtomicBool = AtomicBool::new(false);
static CAMERA_COORD_MUTEX: G8torMutex<(f32, f32, f32)> = G8torMutex::empty();
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
static I2C_MUTEX: G8torMutex<
    I2C<
        I2C1,
        (
            PA6<AlternateFunction<AF3, PushPull>>,
            PA7<AlternateFunction<AF3, OpenDrain<PullUp>>>,
        ),
    >,
> = G8torMutex::empty();

static JOYSTICK_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();
static SPAWNCOOR_FIFO: SyncUnsafeOnceCell<G8torFifoHandle> = SyncUnsafeOnceCell::new();

static JOYSTICK_PUSH_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();
static PCA9555_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();

static CAMERA_COORD_MUT: SyncUnsafeOnceCell<G8torMutexHandle<(f32, f32, f32)>> =
    SyncUnsafeOnceCell::new();
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
static I2C_MUT: SyncUnsafeOnceCell<
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

static mut SPI_BUFFER: [u8; 16] = [0; 16];

extern "C" fn cam_move(_rtos: G8torThreadHandle) -> ! {
    let joystick_fifo_handle = &*JOYSTICK_FIFO;
    let cam_mutex = &*CAMERA_COORD_MUT;

    loop {
        let joystick_val = rtos::read_fifo(joystick_fifo_handle);
        let x = (joystick_val & 0xFFFF) as u16;
        let y = ((joystick_val >> 16) & 0xFFFF) as u16;

        let x = -((x as f32) / (0xFFF as f32) - 0.5);
        let y = (y as f32) / (0xFFF as f32) - 0.5;

        // Deadband
        if x.abs() < 0.05 && y.abs() < 0.05 {
            continue;
        }

        let (y, z) = if JOYSTICK_FLAG.load(core::sync::atomic::Ordering::Relaxed) {
            (0.0, -y)
        } else {
            (y, 0.0)
        };

        // Move camera based on x and y
        let cam_coords = CAMERA_COORD_MUTEX.get(rtos::take_mutex(cam_mutex));

        cam_coords.0 += x;
        cam_coords.1 += y;
        cam_coords.2 += z;

        rtos::release_mutex(cam_mutex, CAMERA_COORD_MUTEX.release(cam_coords));
    }
}

extern "C" fn cube_thread(rtos: G8torThreadHandle) -> ! {
    let cam_mutex = &*CAMERA_COORD_MUT;
    let screen_mutex = &*SCREEN_MUT;
    let spawncoor_fifo = &*SPAWNCOOR_FIFO;

    let x = f32::from_bits(rtos::read_fifo(spawncoor_fifo));
    let y = f32::from_bits(rtos::read_fifo(spawncoor_fifo));
    let z = f32::from_bits(rtos::read_fifo(spawncoor_fifo));

    // Draws and undraws a cube on the screen
    loop {
        let cam_coords = CAMERA_COORD_MUTEX.get(rtos::take_mutex(cam_mutex));
        let (cam_x, cam_y, cam_z) = *cam_coords;
        rtos::release_mutex(cam_mutex, CAMERA_COORD_MUTEX.release(cam_coords));

        let center = (x - cam_x, y - cam_y, -(z - cam_z));

        // Project to 2D screen coordinates (simple perspective projection)
        let cube = Cube {
            center,
            size: 50.0,
            color: Rgb565::GREEN,
        };

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mutex));
        cube.draw(screen).unwrap();
        rtos::release_mutex(screen_mutex, SCREEN_MUTEX.release(screen));

        // Draw cube at (x, y, z)
        rtos::sleep_ms(100);

        // Undraw cube
        let cube = Cube {
            center,
            size: 50.0,
            color: Rgb565::BLACK,
        };
        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mutex));
        cube.draw(screen).unwrap();
        rtos::release_mutex(screen_mutex, SCREEN_MUTEX.release(screen));

        if KILL_CUBE.fetch_and(false, Ordering::Relaxed) {
            rtos.kill();
        };
    }
}

extern "C" fn read_buttons(_rtos: G8torThreadHandle) -> ! {
    let mut i2c = I2C_MUT.clone();
    let spawncoor_fifo = &*SPAWNCOOR_FIFO;
    let mut rand = rand::rngs::SmallRng::seed_from_u64(0x98958219);
    let addr = 0x23;

    loop {
        // Wait for button press (PCA9555 Semaphore)
        rtos::wait_semaphore(&*PCA9555_SEM);
        rtos::sleep_ms(10); // Debounce
        let mut val = [0u8; 1];
        i2c.write_read(addr, &[0x00], &mut val).unwrap();
        let val = val[0];

        // SW1
        if (val & 1 << 1) == 0 {
            // Spawn cube thread
            KILL_CUBE.store(false, Ordering::Relaxed); // Don't kill immediately
            let buff = *b"cube_thread\0\0\0\0\0";
            let x: f32 = rand.random_range(-100.0..100.0);
            let y: f32 = rand.random_range(-100.0..100.0);
            let z: f32 = rand.random_range(-120.0..-20.0);
            rtos::write_fifo(spawncoor_fifo, f32::to_bits(x));
            rtos::write_fifo(spawncoor_fifo, f32::to_bits(y));
            rtos::write_fifo(spawncoor_fifo, f32::to_bits(z));
            let _ = rtos::spawn_thread(&buff, 2, cube_thread);
        }

        // SW2
        if (val & 1 << 2) == 0 {
            // Kill cube thread
            KILL_CUBE.store(true, Ordering::Relaxed);
        }
    }
}

extern "C" fn read_joystick(_rtos: G8torThreadHandle) -> ! {
    // Toggles the joystick flag to determine if Y-axis -> Y/Z
    let joystick_push_sem = &*JOYSTICK_PUSH_SEM;
    loop {
        rtos::wait_semaphore(joystick_push_sem);
        JOYSTICK_FLAG.fetch_xor(true, Ordering::SeqCst);
    }
}

extern "C" fn print_world_coords() {
    // Prints the current world coordinates of the camera every 100ms
    unsafe {
        let coords = CAMERA_COORD_MUTEX.steal();
        let _ = writeln!(
            UART0_S.assume_init_mut(),
            "Camera World Coords: {}, {}, {}",
            coords.0,
            coords.1,
            coords.2
        );
    }
}

extern "C" fn get_joystick() {
    // Read the joystick ADC values and push to FIFO every 100ms
    unsafe {
        let x_adc = JOYSTICK_X_ADC.assume_init_mut();
        let y_adc = JOYSTICK_Y_ADC.assume_init_mut();
        let x_val = x_adc.read();
        let y_val = y_adc.read();
        let joystick_fifo_handle = &*JOYSTICK_FIFO;
        let packed_val: u32 = ((y_val as u32) << 16) | (x_val as u32);
        rtos::write_fifo(joystick_fifo_handle, packed_val);
    }
}

// PORTE Interrupt Handler
extern "C" fn button_isr() {
    // Clear interrupt flag
    let porte = unsafe { &*pac::GPIO_PORTE::ptr() };
    porte.icr.write(|w| unsafe { w.gpio().bits(1 << 4) });

    // Signal PCA9555 Semaphore
    rtos::signal_semaphore(&*PCA9555_SEM);
}

// PORTD Interrupt Handler
extern "C" fn joystick_click_isr() {
    // Clear interrupt flag
    let portd = unsafe { &*pac::GPIO_PORTD::ptr() };
    portd.icr.write(|w| unsafe { w.gpio().bits(1 << 2) });

    // Toggle joystick flag
    JOYSTICK_FLAG.fetch_xor(true, Ordering::Relaxed);
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
    let portd = p.GPIO_PORTD.split(&sc.power_control);
    let porte = p.GPIO_PORTE.split(&sc.power_control);
    let portf = p.GPIO_PORTF.split(&sc.power_control);

    // Enable buttons interrupt
    let mut pe4 = porte.pe4.into_pull_up_input();
    pe4.set_interrupt_mode(hal::gpio::InterruptMode::EdgeFalling);

    // Initialize Joystick ADCs
    let pe3 = porte.pe3.into_floating_input().into_analog();
    let pe2 = porte.pe2.into_floating_input().into_analog();

    let adc_x = hal::adc::AdcSingle::adc0(p.ADC0, pe3, &sc.power_control);
    let adc_y = hal::adc::AdcSingle::adc1(p.ADC1, pe2, &sc.power_control);

    // Enable Joystick Interrupt
    let mut pd2 = portd.pd2.into_pull_up_input();
    pd2.set_interrupt_mode(hal::gpio::InterruptMode::EdgeFalling);

    // Activate UART
    let uart = hal::serial::Serial::uart0(
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

    let joystick_fifo = inst.init_fifo().expect("We haven't run out of atomics");
    let spawncoor_fifo = inst.init_fifo().expect("We haven't run out of atomics");

    let joystick_push_sem = inst
        .init_semaphore(0)
        .expect("We haven't run out of atomics");
    let pca9555_sem = inst
        .init_semaphore(0)
        .expect("We haven't run out of atomics");

    let camera_coord_mut = inst
        .init_mutex(&CAMERA_COORD_MUTEX)
        .expect("We haven't run out of atomics");
    let screen_mut = inst
        .init_mutex(&SCREEN_MUTEX)
        .expect("We haven't run out of atomics");
    let i2c_mut = inst
        .init_mutex(&I2C_MUTEX)
        .expect("We haven't run out of atomics");

    inst.add_thread(&byte_str("cam_move"), 1, cam_move)
        .expect("TCB list has space");
    inst.add_thread(&byte_str("read_buttons"), 1, read_buttons)
        .expect("TCB list has space");
    inst.add_thread(&byte_str("read_joystick"), 1, read_joystick)
        .expect("TCB list has space");

    inst.add_periodic(100, 0, print_world_coords)
        .expect("Periodic TCB list has space");
    inst.add_periodic(100, 50, get_joystick)
        .expect("Periodic TCB list has space");

    inst.add_event(pac::interrupt::GPIOD, 5, joystick_click_isr)
        .expect("Inputs are correct");
    inst.add_event(pac::interrupt::GPIOE, 5, button_isr)
        .expect("Inputs are correct");

    unsafe {
        UART0_S.write(uart);
        JOYSTICK_X_ADC.write(adc_x);
        JOYSTICK_Y_ADC.write(adc_y);

        CAMERA_COORD_MUTEX.init((0.0, 0.0, 0.0));
        SCREEN_MUTEX.init(display);
        I2C_MUTEX.init(i2c);

        JOYSTICK_FIFO.set(joystick_fifo);
        SPAWNCOOR_FIFO.set(spawncoor_fifo);

        JOYSTICK_PUSH_SEM.set(joystick_push_sem);
        PCA9555_SEM.set(pca9555_sem);

        CAMERA_COORD_MUT.set(camera_coord_mut);
        SCREEN_MUT.set(screen_mut);
        I2C_MUT.set(i2c_mut);

        inst.launch()
    }
}
