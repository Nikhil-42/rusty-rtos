#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::fmt::Write as _;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};
use eel4745c::SyncUnsafeOnceCell;

use eh0::serial::{Read as _, Write as _};
use embedded_graphics::{
    pixelcolor::Rgb565,
};
use embedded_hal::{digital::OutputPin, i2c::I2c, spi::SpiBus};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_graphics::{
    prelude::*,
    primitives::*
};
use mipidsi::Display;
use mipidsi::{interface::SpiInterface, Builder};

use eel4745c::rtos::{
    self, G8torFifoHandle, G8torMutex, G8torMutexHandle, G8torSemaphoreHandle, G8torThreadHandle,
};
use panic_halt as _;

use rand::{RngCore, SeedableRng};
use tm4c123x_hal::gpio::gpioa::{PA6, PA7};
use tm4c123x_hal::gpio::{
    gpioa::{PA2, PA4, PA5},
    gpioe::PE0,
    gpiof::{PF3, PF4},
    Floating, OpenDrain, Output, AF2,
};
use tm4c123x_hal::gpio::{PullUp, AF3};
use tm4c123x_hal::i2c::I2C;
use tm4c123x_hal::pac::I2C1;
use tm4c123x_hal::spi::Spi;
use tm4c123x_hal::{
    self as hal,
    gpio::{
        gpioa::{PA0, PA1},
        AlternateFunction, PushPull, AF1,
    },
    pac::{self, SSI0, UART0},
    prelude::*,
    // prelude::,
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

#[inline(never)]
extern "C" fn cam_move(_rtos: G8torThreadHandle) -> ! {
    let joystick_fifo_handle = &*JOYSTICK_FIFO;
    let cam_mutex = &*CAMERA_COORD_MUT;

    loop {
        let joystick_val = rtos::read_fifo(joystick_fifo_handle);
        let x = (joystick_val & 0xFF) as u16;
        let y = ((joystick_val >> 8) & 0xFF) as u16;

        let x = x as f32 - 128.0 / 128.0;
        let y = y as f32 - 128.0 / 128.0;
        let (y, z) = if JOYSTICK_FLAG.load(core::sync::atomic::Ordering::Relaxed) {
            (y, 0.0)
        } else {
            (0.0, y)
        };

        // Move camera based on x and y
        let cam_coords = CAMERA_COORD_MUTEX.get(rtos::take_mutex(cam_mutex));

        cam_coords.0 += x;
        cam_coords.1 += y;
        cam_coords.2 += z;

        rtos::release_mutex(cam_mutex, CAMERA_COORD_MUTEX.release(cam_coords));
    }
}

#[inline(never)]
extern "C" fn cube_thread(rtos: G8torThreadHandle) -> ! {
    // let cam_mutex = &*CAMERA_COORD_MUT;
    let screen_mutex = &*SCREEN_MUT;
    let spawncoor_fifo = &*SPAWNCOOR_FIFO;

    let coord_val = rtos::read_fifo(spawncoor_fifo);
    let x = (coord_val >> 16) as i32;
    let y = (coord_val & 0xFFFF) as i32;

    // Draws and undraws a cube on the screen
    loop {
        // let cam_coords = CAMERA_COORD_MUTEX.get(rtos::take_mutex(cam_mutex));
        // let (x, y, z) = *cam_coords;
        // rtos::release_mutex(cam_mutex, CAMERA_COORD_MUTEX.release(cam_coords));

        let rect = Rectangle::new(
            Point::new(x, y),
            Size::new(50, 50),
        );
        let rect_styled = rect.into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1));

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mutex));
        rect_styled.draw(screen).unwrap();
        rtos::release_mutex(screen_mutex, SCREEN_MUTEX.release(screen));

        // Draw cube at (x, y, z)
        rtos::sleep_ms(1000);

        // Undraw cube
        let undraw = rect.into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1));
        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mutex));
        undraw.draw(screen).unwrap();
        rtos::release_mutex(screen_mutex, SCREEN_MUTEX.release(screen));

        if KILL_CUBE.fetch_and(false, Ordering::Relaxed) {
            rtos.kill();
        };
            
    }
}

#[inline(never)]
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
            let buff = *b"cube_thread\0\0\0\0\0";
            let x = rand.next_u32() % 256;
            let y = rand.next_u32() % 256;
            rtos::write_fifo(spawncoor_fifo, x << 16 | y);
            let _ = rtos::spawn_thread(&buff, 2, cube_thread);
        }

        // SW2
        if (val & 1 << 2) == 0 {
            // Kill cube thread
            KILL_CUBE.store(true, Ordering::Relaxed);
        }
    }
}

#[inline(never)]
extern "C" fn read_joystick(_rtos: G8torThreadHandle) -> ! {
    // Toggles the joystick flag to determine if Y-axis -> Y/Z
    let joystick_push_sem = &*JOYSTICK_PUSH_SEM;
    loop {
        rtos::wait_semaphore(joystick_push_sem);
        JOYSTICK_FLAG.fetch_xor(true, Ordering::SeqCst);
    }
}

#[inline(never)]
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

#[inline(never)]
extern "C" fn get_joystick() {
    // Read the joystick ADC values and push to FIFO every 100ms
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
    // let pe2, pe3

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
        .orientation(mipidsi::options::Orientation::new().flip_vertical())
        .reset_pin(tft_rst)
        .init(&mut delay_source)
        .unwrap();
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

    inst.add_thread(b"cam_move\0\0\0\0\0\0\0\0", 1, cam_move)
        .expect("TCB list has space");
    inst.add_thread(b"read_buttons\0\0\0\0", 1, read_buttons)
        .expect("TCB list has space");
    inst.add_thread(b"read_joystick\0\0\0", 1, read_joystick)
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
