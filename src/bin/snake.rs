#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::fmt::Write as _;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};
use eel4745c::SyncUnsafeOnceCell;

use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle, StyledDrawable};
use embedded_graphics::text::{Alignment, Baseline, LineHeight, Text, TextStyleBuilder};
use embedded_hal::digital::OutputPin;
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

static DIE: AtomicBool = AtomicBool::new(false);

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
static DIRECTION_MUTEX: G8torMutex<Direction> = G8torMutex::empty();
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

static RESTART_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();
static SCORE_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();

static DIRECTION_MUT: SyncUnsafeOnceCell<G8torMutexHandle<Direction>> = SyncUnsafeOnceCell::new();
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

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
enum Direction {
    Up,
    Down,
    Left,
    Right,
}

impl Direction {
    fn push(&self, point: (i32, i32)) -> (i32, i32) {
        match self {
            Direction::Up => (point.0, point.1 - 1),
            Direction::Down => (point.0, point.1 + 1),
            Direction::Left => (point.0 - 1, point.1),
            Direction::Right => (point.0 + 1, point.1),
        }
    }
}

const SCALE: u32 = 10;

static mut SPI_BUFFER: [u8; 16] = [0; 16];

extern "C" fn input_direction(_rtos: G8torThreadHandle) -> ! {
    let joystick_fifo_handle = &*JOYSTICK_FIFO;
    let cam_mutex = &*DIRECTION_MUT;

    loop {
        let joystick_val = rtos::read_fifo(joystick_fifo_handle);
        let x = (joystick_val & 0xFFFF) as u16;
        let y = ((joystick_val >> 16) & 0xFFFF) as u16;

        let x = -((x as f32) / (0x7FF as f32) - 1.0); // -1 to 1
        let y = (y as f32) / (0x7FF as f32) - 1.0; // -1 to 1

        let direction = DIRECTION_MUTEX.get(rtos::take_mutex(cam_mutex));
        if y.abs() < 0.5 {
            if x > 0.5 {
                // Move right
                *direction = if *direction != Direction::Left {
                    Direction::Right
                } else {
                    Direction::Left
                };
            } else if x < -0.5 {
                // Move left
                *direction = if *direction != Direction::Right {
                    Direction::Left
                } else {
                    Direction::Right
                };
            }
        } else if x.abs() < 0.5 {
            if y > 0.5 {
                // Move up
                *direction = if *direction != Direction::Down {
                    Direction::Up
                } else {
                    Direction::Down
                };
            } else if y < -0.5 {
                // Move down
                *direction = if *direction != Direction::Up {
                    Direction::Down
                } else {
                    Direction::Up
                };
            }
        }
        rtos::release_mutex(cam_mutex, DIRECTION_MUTEX.release(direction));
    }
}

extern "C" fn snake_thread(rtos: G8torThreadHandle) -> ! {
    let direction_mutex = &*DIRECTION_MUT;
    let screen_mutex = &*SCREEN_MUT;
    let restart_sem = &*RESTART_SEM;
    let score_sem = &*SCORE_SEM;

    let mut chain = [Direction::Right; 64]; // Ring buffer for snake directions
    let mut length = 4; // Initial length of snake
    let mut head_ptr = length-1; // Empty slot
    let mut tail_ptr = 0; // Points to tail

    let mut pos = (8, 16);
    let mut tail = (pos.0 - length as i32 + 1, 16);

    let mut rand = rand::rngs::SmallRng::seed_from_u64(0x2007_FFFC);
    let mut fruit = (rand.random_range(0..24), rand.random_range(4..28));

    let fruit_square = Rectangle::new(
        Point::new(fruit.0 * SCALE as i32, fruit.1 * SCALE as i32),
        Size::new(SCALE, SCALE),
    );
    let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mutex));
    fruit_square
        .draw_styled(
            &PrimitiveStyleBuilder::new().fill_color(Rgb565::RED).build(),
            screen,
        )
        .unwrap();
    rtos::release_mutex(screen_mutex, SCREEN_MUTEX.release(screen));

    let erase = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();
    let draw = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::GREEN)
        .build();
    let red = PrimitiveStyleBuilder::new().fill_color(Rgb565::RED).build();

    // Draws and undraws a cube on the screen
    loop {
        if DIE.load(Ordering::Relaxed) {
            // Confirm death
            rtos::signal_semaphore(restart_sem);
            rtos.kill();
        }

        let direction = DIRECTION_MUTEX.get(rtos::take_mutex(direction_mutex));
        let dir = *direction;
        rtos::release_mutex(direction_mutex, DIRECTION_MUTEX.release(direction));

        let next_pos = dir.push(pos);
        let next_tail = chain[tail_ptr].push(tail);
        chain[head_ptr] = dir;

        let head_square = Rectangle::new(
            Point::new(next_pos.0 * SCALE as i32, next_pos.1 * SCALE as i32),
            Size::new(SCALE, SCALE),
        );
        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mutex));
        head_square.draw_styled(&draw, screen).unwrap();
        if next_pos.0 >= 24
            || next_pos.1 >= 30 
            || next_pos.0 < 0
            || next_pos.1 < 2
        {
            // Out of bounds, die
            rtos::release_mutex(screen_mutex, SCREEN_MUTEX.release(screen));
            // Trigger restart
            rtos::signal_semaphore(restart_sem);
            // Confirm death
            rtos::signal_semaphore(restart_sem);
            rtos.kill();
        } else if (0..length).map(|i| chain[(tail_ptr + i) % chain.len()])
                .scan(tail, |state, d| {
                    let current = *state;
                    *state= d.push(current);
                    Some(current)
                })
                .any(|p| p == next_pos)
        {
            // Self, turn red
            for pos in (0..length).map(|i| chain[(tail_ptr + i) % chain.len()])
                .scan(tail, |state, d| {
                    let current = *state;
                    *state= d.push(current);
                    Some(current)
                })
            {
                let square = Rectangle::new(
                    Point::new(pos.0 * SCALE as i32, pos.1 * SCALE as i32),
                    Size::new(SCALE, SCALE),
                );
                square.draw_styled(&red, screen).unwrap();
            }
            rtos::release_mutex(screen_mutex, SCREEN_MUTEX.release(screen));
            // Trigger restart
            rtos::signal_semaphore(restart_sem);
            // Confirm death
            rtos::signal_semaphore(restart_sem);
            rtos.kill();
        } else if next_pos == fruit {
            // Ate fruit, grow snake
            length += 1;
            rtos::signal_semaphore(score_sem);
            fruit = (rand.random_range(0..24), rand.random_range(4..28));
            // Draw new fruit
            let fruit_square = Rectangle::new(
                Point::new(fruit.0 * SCALE as i32, fruit.1 * SCALE as i32),
                Size::new(SCALE, SCALE),
            );
            fruit_square.draw_styled(&red, screen).unwrap();
        } else {
            // Erase tail
            let tail_square = Rectangle::new(
                Point::new(tail.0 * SCALE as i32, tail.1 * SCALE as i32),
                Size::new(SCALE, SCALE),
            );
            tail_square.draw_styled(&erase, screen).unwrap();
            tail = next_tail;

            tail_ptr = (tail_ptr + 1) % chain.len();
        }
        head_ptr = (head_ptr + 1) % chain.len();

        rtos::release_mutex(screen_mutex, SCREEN_MUTEX.release(screen));

        pos = next_pos;
        rtos::sleep_ms(1000);
    }
}

extern "C" fn restart_game(_rtos: G8torThreadHandle) -> ! {
    // If the joystick is pushed restart the game
    let restart_sem = &*RESTART_SEM;
    let direction_mut = &*DIRECTION_MUT;
    let screen_mut = &*SCREEN_MUT;
    loop {
        rtos::wait_semaphore(restart_sem);
        DIE.store(true, Ordering::Relaxed);
        rtos::wait_semaphore(restart_sem);
        DIE.store(false, Ordering::Relaxed);

        rtos::sleep_ms(3000);  // show death for 3 seconds
        let direction = DIRECTION_MUTEX.get(rtos::take_mutex(direction_mut));
        *direction = Direction::Right;
        rtos::release_mutex(direction_mut, DIRECTION_MUTEX.release(direction));

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        screen.clear(Rgb565::BLACK).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));

        rtos::spawn_thread(b"snake\0\0\0\0\0\0\0\0\0\0\0", 1, snake_thread);
    }
}

extern "C" fn show_score(_rtos: G8torThreadHandle) -> ! {
    let score_sem = &*SCORE_SEM;
    let screen_mut = &*SCREEN_MUT;
    let text_style = TextStyleBuilder::new()
        .line_height(LineHeight::Pixels(30)).alignment(Alignment::Left).baseline(Baseline::Top).build();
    let char_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    let mut score = 0;
    let mut score_text_buf: heapless::String<4> = heapless::String::new();
    loop {
        rtos::wait_semaphore(score_sem);
        score += 1;

        write!(score_text_buf, "{}", score).unwrap();
        let mut score_text = Text::new(&score_text_buf, Point::new(10, 10), char_style);
        score_text.text_style = text_style;
        let score_text = score_text;
        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        score_text.draw(screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));
        score_text_buf.clear();
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

// PORTD Interrupt Handler
extern "C" fn joystick_click_isr() {
    // Clear interrupt flag
    let portd = unsafe { &*pac::GPIO_PORTD::ptr() };
    portd.icr.write(|w| unsafe { w.gpio().bits(1 << 2) });

    // Signal to restart the game
    let restart_sem = &*RESTART_SEM;
    rtos::signal_semaphore(restart_sem);
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

    let inst = unsafe { rtos::G8torRtos::new(core_p) };

    let joystick_fifo = inst.init_fifo().expect("We haven't run out of atomics");
    let spawncoor_fifo = inst.init_fifo().expect("We haven't run out of atomics");

    let joystick_push_sem = inst
        .init_semaphore(0)
        .expect("We haven't run out of atomics");
    let pca9555_sem = inst
        .init_semaphore(0)
        .expect("We haven't run out of atomics");

    let direction_mut = inst
        .init_mutex(&DIRECTION_MUTEX)
        .expect("We haven't run out of atomics");
    let screen_mut = inst
        .init_mutex(&SCREEN_MUTEX)
        .expect("We haven't run out of atomics");
    let i2c_mut = inst
        .init_mutex(&I2C_MUTEX)
        .expect("We haven't run out of atomics");

    inst.add_thread(b"input_conversion", 1, input_direction)
        .expect("TCB list has space");
    inst.add_thread(b"read_joystick\0\0\0", 1, restart_game)
        .expect("TCB list has space");
    inst.add_thread(b"snake\0\0\0\0\0\0\0\0\0\0\0", 1, snake_thread)
        .expect("TCB list has space") as u32;
    inst.add_thread(b"show_score\0\0\0\0\0\0", 1, show_score)
        .expect("TCB list has space");

    inst.add_event(pac::Interrupt::GPIOD, 2, joystick_click_isr).unwrap();

    inst.add_periodic(100, 50, get_joystick)
        .expect("Periodic TCB list has space");

    unsafe {
        UART0_S.write(uart);
        JOYSTICK_X_ADC.write(adc_x);
        JOYSTICK_Y_ADC.write(adc_y);

        DIRECTION_MUTEX.init(Direction::Right);
        SCREEN_MUTEX.init(display);
        I2C_MUTEX.init(i2c);

        JOYSTICK_FIFO.set(joystick_fifo);
        SPAWNCOOR_FIFO.set(spawncoor_fifo);

        RESTART_SEM.set(joystick_push_sem);
        SCORE_SEM.set(pca9555_sem);

        DIRECTION_MUT.set(direction_mut);
        SCREEN_MUT.set(screen_mut);
        I2C_MUT.set(i2c_mut);

        inst.launch()
    }
}
