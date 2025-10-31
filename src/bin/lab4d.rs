#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::{iter, mem::MaybeUninit};
use core::sync::atomic::{AtomicBool, Ordering};
use core::fmt::Write as _;
use cortex_m::delay;
use eel4745c::SyncUnsafeOnceCell;

use embedded_graphics::{prelude::*, mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder}, pixelcolor::Rgb565, text::Text, primitives::Rectangle};
use mipidsi::Builder;

use eel4745c::rtos::{self, G8torFifoHandle, G8torMutex, G8torMutexHandle, G8torThreadHandle, G8torSemaphoreHandle};
use eh0::digital::v2::OutputPin as _;
use panic_halt as _;

use tm4c123x_hal::{gpio::{Floating, GpioExt as _}, sysctl::SysctlExt as _, time::U32Ext as _, tm4c123x::adc0::dccmp0::R};
use tm4c123x_hal::{
    self as hal,
    gpio::{
        AF1, AlternateFunction, PushPull, gpioa::{PA0, PA1}
    },
    tm4c123x::{self as pac, UART0, SSI0},
    // prelude::,
    serial::Serial,
};

use cortex_m_rt::entry;

static mut JOYSTICK_FLAG: AtomicBool = AtomicBool::new(false);

static mut UART0_S: MaybeUninit<
    Serial<
        UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
> = MaybeUninit::uninit();
static CAMERA_COORD_MUTEX: G8torMutex<
    (f32, f32, f32),
> = G8torMutex::empty();
static SCREEN_MUTEX: G8torMutex<
    mipidsi::Display<display_interface_spi::SPIInterface<tm4c123x_hal::spi::Spi<SSI0, (tm4c123x_hal::gpio::gpioa::PA2<AlternateFunction<tm4c123x_hal::gpio::AF2, PushPull>>, tm4c123x_hal::gpio::gpioa::PA4<AlternateFunction<tm4c123x_hal::gpio::AF2, PushPull>>, tm4c123x_hal::gpio::gpioa::PA5<AlternateFunction<tm4c123x_hal::gpio::AF2, tm4c123x_hal::gpio::OpenDrain<Floating>>>)>, tm4c123x_hal::gpio::gpiof::PF3<tm4c123x_hal::gpio::Output<PushPull>>, tm4c123x_hal::gpio::gpiof::PF4<tm4c123x_hal::gpio::Output<PushPull>>>, mipidsi::models::ST7789, tm4c123x_hal::gpio::gpioe::PE0<tm4c123x_hal::gpio::Output<PushPull>>>
> = G8torMutex::empty();

static JOYSTICK_FIFO: SyncUnsafeOnceCell<
    G8torFifoHandle
> = SyncUnsafeOnceCell::new();
static SPAWNCOOR_FIFO: SyncUnsafeOnceCell<
    G8torFifoHandle
> = SyncUnsafeOnceCell::new();

static JOYSTICK_PUSH_SEM: SyncUnsafeOnceCell<
    G8torSemaphoreHandle
> = SyncUnsafeOnceCell::new();
static PCA9555_SEM: SyncUnsafeOnceCell<
    G8torSemaphoreHandle
> = SyncUnsafeOnceCell::new();

static CAMERA_COORD_MUT: SyncUnsafeOnceCell<
    G8torMutexHandle<(f32, f32, f32)>
> = SyncUnsafeOnceCell::new();
static SCREEN_MUT: SyncUnsafeOnceCell<
    G8torMutexHandle<mipidsi::Display<display_interface_spi::SPIInterface<tm4c123x_hal::spi::Spi<SSI0, (tm4c123x_hal::gpio::gpioa::PA2<AlternateFunction<tm4c123x_hal::gpio::AF2, PushPull>>, tm4c123x_hal::gpio::gpioa::PA4<AlternateFunction<tm4c123x_hal::gpio::AF2, PushPull>>, tm4c123x_hal::gpio::gpioa::PA5<AlternateFunction<tm4c123x_hal::gpio::AF2, tm4c123x_hal::gpio::OpenDrain<Floating>>>)>, tm4c123x_hal::gpio::gpiof::PF3<tm4c123x_hal::gpio::Output<PushPull>>, tm4c123x_hal::gpio::gpiof::PF4<tm4c123x_hal::gpio::Output<PushPull>>>, mipidsi::models::ST7789, tm4c123x_hal::gpio::gpioe::PE0<tm4c123x_hal::gpio::Output<PushPull>>>>
> = SyncUnsafeOnceCell::new();

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
        let (y, z) = if unsafe { JOYSTICK_FLAG.load(core::sync::atomic::Ordering::Relaxed) } {
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
extern "C" fn cube_thread(_rtos: G8torThreadHandle) -> ! {
    let cam_mutex = &*CAMERA_COORD_MUT;
    let screen_mutex = &*SCREEN_MUT;

    // Draws and undraws a cube on the screen
    loop {
        let cam_coords = CAMERA_COORD_MUTEX.get(rtos::take_mutex(cam_mutex));
        let (x, y, z) = *cam_coords;
        rtos::release_mutex(cam_mutex, CAMERA_COORD_MUTEX.release(cam_coords));

        let rect = Rectangle::new(Point::new((x * 10.0) as i32, (y * 10.0) as i32), Size::new(50, 50));

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mutex));
        screen.fill_contiguous(&rect, core::iter::repeat_with(|| Rgb565::WHITE).take((rect.size.width * rect.size.height) as usize)).unwrap();
        rtos::release_mutex(screen_mutex, SCREEN_MUTEX.release(screen));

        // Draw cube at (x, y, z)
        rtos::sleep_ms(500);
        // Undraw cube

    }
}

#[inline(never)]
extern "C" fn read_buttons(_rtos: G8torThreadHandle) -> ! {
    // Wait for PCA9555 Semaphore
    // Sleep
    // Chceck buttons

    // if SW1 pressed 
    // Spawn cube thread

    // if SW2 pressed
    // Kill cube thread if exists
    loop {
        // let val = rtos.read_fifo(rx_fifo_handle);
        // if rtos.kill_thread(val as usize) == 1 {
        //     // Failed to kill thread
        //     // Spawn a new publisher thread
        //     let mut buff = *b"publish\0\0\0\0\0\0\0\0\0";
        //     buff[7] = val as u8 + b'0';
        //     rtos.spawn_thread(&buff, 2, publisher);
        // }
    }
}

#[inline(never)]
extern "C" fn read_joystick_press(_rtos: G8torThreadHandle) -> ! {
    // Toggles the joystick flag to determine if Y-axis -> Y/Z
    let joystick_push_sem = &*JOYSTICK_PUSH_SEM;
    loop {
        rtos::wait_semaphore(joystick_push_sem);
        unsafe { JOYSTICK_FLAG.fetch_xor(true, Ordering::SeqCst); }
    }
}

#[inline(never)]
extern "C" fn print_world_coords() {
    // Prints the current world coordinates of the camera every 100ms
    unsafe {
        let coords = CAMERA_COORD_MUTEX.steal();
        let _ = writeln!(UART0_S.assume_init_mut(), "Camera World Coords: {}, {}, {}", coords.0, coords.1, coords.2);
    }
}

#[inline(never)]
extern "C" fn get_joystick() {

}

extern "C" fn gpioe_handler() {
    // Clear interrupt flag
    cortex_m::asm::nop();

    // Signal PCA9555 Semaphore
}

extern "C" fn gpiod_handler() {
    // Clear interrupt flag

    // Read joystick position from PCA9555

    // Write joystick position to FIFO
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
    let mut pd2 = portd.pd6.into_pull_up_input();
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

    // Activate SPI module
    let spi = hal::spi::Spi::<SSI0, _>::spi0(
        p.SSI0,
        (
            porta.pa2.into_af_push_pull::<hal::gpio::AF2>(&mut porta.control),
            porta.pa4.into_af_push_pull::<hal::gpio::AF2>(&mut porta.control),
            porta.pa5.into_af_open_drain::<hal::gpio::AF2, Floating>(&mut porta.control),
        ),
        eh0::spi::MODE_3,
        25.mhz(),
        &clocks,
        &sc.power_control,
    );

    let mut tft_rst = porte.pe0.into_push_pull_output();
    tft_rst.set_high().unwrap();
    let tft_dc = portf.pf3.into_push_pull_output();
    let tft_cs = portf.pf4.into_push_pull_output();

    let di = display_interface_spi::SPIInterface::new(spi, tft_dc, tft_cs);

    let mut core_p = pac::CorePeripherals::take().unwrap();

    let mut delay_source = hal::delay::Delay::new(core_p.SYST, &clocks);
    let mut display = Builder::st7789(di)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .with_orientation(mipidsi::Orientation::PortraitInverted(false))
        .init(&mut delay_source, Some(tft_rst))
        .unwrap();
    core_p.SYST = delay_source.free();
    display.clear(Rgb565::BLACK).unwrap();

    unsafe {
        UART0_S.write(uart);
        SCREEN_MUTEX.init(display);
    }

    let inst = unsafe {
        rtos::G8torRtos::new(core_p)
    };

    let joystick_fifo = inst.init_fifo().expect("We haven't run out of atomics");
    let joystick_push_sem = inst.init_semaphore(0).expect("We haven't run out of atomics");

    inst.add_thread(b"cam_move\0\0\0\0\0\0\0\0", 1, cam_move).expect("TCB list has space");
    inst.add_thread(b"read_buttons\0\0\0\0", 1, read_buttons).expect("TCB list has space");
    inst.add_thread(b"read_joystick\0\0\0", 1, read_joystick_press).expect("TCB list has space");

    inst.add_periodic(100, 1, print_world_coords).expect("Periodic TCB list has space");
    inst.add_periodic(100, 50, get_joystick).expect("Periodic TCB list has space");

    inst.add_event(pac::interrupt::GPIOE, 5, gpioe_handler).expect("Inputs are correct");
    inst.add_event(pac::interrupt::GPIOD, 5, gpiod_handler).expect("Inputs are correct");

    unsafe {
        JOYSTICK_FIFO.set(joystick_fifo);
        JOYSTICK_PUSH_SEM.set(joystick_push_sem);
        inst.launch()
    }
}
