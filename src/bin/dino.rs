#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, iso_8859_1::FONT_10X20}, pixelcolor::Rgb565, prelude::*, primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StyledDrawable as _}, text::Text
};
use panic_halt as _;

use eel4745c::{
    byte_str,
    game::*,
    physics::Direction,
    rtos::{self, G8torMutex, G8torMutexHandle, G8torSemaphoreHandle, G8torThreadHandle},
    SyncUnsafeOnceCell,
};
use rand::{Rng, SeedableRng};
use core::fmt::Write as _;
use micromath::F32Ext as _;

// Constants
const DINO_SIZE: (u32, u32) = (4, 10);
const CACTUS_SIZE: (u32, u32) = (2, 10);
const LINE_Y: i32 = 240;
const PLAYER_X: i32 = 40;
const FALL_SPEED: f32 = 15.0;

const ERASE: PrimitiveStyle<Rgb565> = PrimitiveStyleBuilder::new()
    .fill_color(Rgb565::BLACK)
    .build();

// Atomics
static DIE: AtomicBool = AtomicBool::new(false);

// Shared Resources
static PLAYER_Y_MUTEX: G8torMutex<f32> = G8torMutex::empty();
static PLAYER_SPEED_MUTEX: G8torMutex<f32> = G8torMutex::empty();
static DIRECTION_MUTEX: G8torMutex<Option<Direction>> = G8torMutex::empty();

// Mutex Handles
static PLAYER_Y_MUT: SyncUnsafeOnceCell<G8torMutexHandle<f32>> = SyncUnsafeOnceCell::new();
static PLAYER_SPEED_MUT: SyncUnsafeOnceCell<G8torMutexHandle<f32>> = SyncUnsafeOnceCell::new();
static DIRECTION_MUT: SyncUnsafeOnceCell<G8torMutexHandle<Option<Direction>>> =
    SyncUnsafeOnceCell::new();

// FIFOs

// Semaphore Handles
static RESTART_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();
static SCORE_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();

// Threads
extern "C" fn input_direction(_rtos: G8torThreadHandle) -> ! {
    loop {
        let (x, y) = get_joystick();

        let direction = DIRECTION_MUTEX.get(rtos::take_mutex(&*DIRECTION_MUT));
        *direction = if x.abs() < 0.5 && y.abs() > 0.5 {
            if y > 0.5 {
                Some(Direction::Up)
            } else {
                Some(Direction::Down)
            }
        } else {
            None
        };
        rtos::release_mutex(&*DIRECTION_MUT, DIRECTION_MUTEX.release(direction));
    }
}

extern "C" fn restart_game(_rtos: G8torThreadHandle) -> ! {
    let restart_sem = &*RESTART_SEM;
    let screen_mut = &*SCREEN_MUT;

    loop {
        let rect = Rectangle::new(
            Point::new(0, LINE_Y),
            Size::new(240, 1),
        );
        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        screen.clear(Rgb565::BLACK).unwrap();
        rect.draw_styled(&PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build(), screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));

        rtos::spawn_thread(&byte_str("cactus_generator"), 2, cactus_generator);
        rtos::spawn_thread(&byte_str("dino"), 1, dino_thread);
        rtos::wait_semaphore(restart_sem);
        DIE.store(true, Ordering::Relaxed);
        rtos::kill_thread(0x1);
        rtos::sleep_ms(5000);

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        screen.clear(Rgb565::RED).unwrap();
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::WHITE)
            .build();

        Text::new("Game Over", Point::new(90, 120), text_style)
            .draw(screen)
            .unwrap();

        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));
        
        let uart = UART_MUTEX.get(rtos::take_mutex(&*UART_MUT));
        writeln!(uart, "You Lost").unwrap();
        rtos::release_mutex(&*UART_MUT, UART_MUTEX.release(uart));

        rtos::wait_semaphore(restart_sem);
        DIE.store(false, Ordering::SeqCst);
    }
}

extern "C" fn cactus_generator(_rtos: G8torThreadHandle) -> ! {
    let mut rand = rand::rngs::SmallRng::seed_from_u64(rtos::get_system_time() as u64);

    loop {
        // Sleep between 1 and 10 seconds
        let sleep_time = rand.random_range(1000..10000);
        rtos::sleep_ms(sleep_time);

        rtos::spawn_thread(&byte_str("cactus"), 1, cactus_thread);
    }
}

extern "C" fn cactus_thread(rtos: G8torThreadHandle) -> ! {
    let screen_mut = &*SCREEN_MUT;
    let player_y_mut = &*PLAYER_Y_MUT;
    let player_speed_mut = &*PLAYER_SPEED_MUT;

    // let mut rand = rand::rngs::SmallRng::seed_from_u64(rtos::get_system_time() as u64);
    let mut x_pos = 240.0f32;
    let height = 10;
    let draw = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::GREEN)
        .build();

    loop {
        if DIE.load(Ordering::Relaxed) {
            rtos.kill();
        }

        let speed = PLAYER_SPEED_MUTEX.get(rtos::take_mutex(player_speed_mut));
        let next_x_pos = x_pos - *speed * 0.1;
        rtos::release_mutex(player_speed_mut, PLAYER_SPEED_MUTEX.release(speed));

        // Collision Detection
        if x_pos >= (PLAYER_X + DINO_SIZE.1 as i32) as f32
            && next_x_pos <= (PLAYER_X + DINO_SIZE.1 as i32) as f32
        {
            // We passed through the players position
            let player_y_ref = PLAYER_Y_MUTEX.get(rtos::take_mutex(player_y_mut));
            let player_y = *player_y_ref;
            rtos::release_mutex(player_y_mut, PLAYER_Y_MUTEX.release(player_y_ref));

            if player_y < height as f32 {
                // The player dies
                DIE.store(true, Ordering::SeqCst);
            } else {
                // Increase speed for sucessfully avoided cacti
                let player_speed_ref = PLAYER_SPEED_MUTEX.get(rtos::take_mutex(player_speed_mut));
                *player_speed_ref += 1.0;
                rtos::release_mutex(
                    player_speed_mut,
                    PLAYER_SPEED_MUTEX.release(player_speed_ref),
                );
            }
        }

        // If we go off-screen free up
        if x_pos + (CACTUS_SIZE.0 as f32) < 0.0 {
            rtos.kill();
        }

        let rect = Rectangle::new(
            Point::new(x_pos as i32, LINE_Y - height),
            Size::new(CACTUS_SIZE.0, height as u32),
        );

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        rect.draw_styled(&draw, screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));

        rtos::sleep_ms(100);

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        rect.draw_styled(&ERASE, screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));

        x_pos = next_x_pos;
    }
}

extern "C" fn dino_thread(rtos: G8torThreadHandle) -> ! {
    let player_y_mut = &*PLAYER_Y_MUT;
    let screen_mut = &*SCREEN_MUT;
    let direction_mut = &*DIRECTION_MUT;

    let draw = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::WHITE)
        .build();

    loop {
        if DIE.load(core::sync::atomic::Ordering::Relaxed) {
            // Confirm death
            rtos::signal_semaphore(&*&RESTART_SEM);
            rtos.kill();
        }

        let direction = DIRECTION_MUTEX.get(rtos::take_mutex(direction_mut));
        let dir = *direction;
        rtos::release_mutex(direction_mut, DIRECTION_MUTEX.release(direction));

        let player_y = PLAYER_Y_MUTEX.get(rtos::take_mutex(player_y_mut));
        let current_y = *player_y;
        rtos::release_mutex(player_y_mut, PLAYER_Y_MUTEX.release(player_y));

        let next_y = if current_y <= 0.0 && dir == Some(Direction::Up) {
            // Jump to a height of 50 over 1000ms
            for i in 0..9 {
                let player_y = PLAYER_Y_MUTEX.get(rtos::take_mutex(player_y_mut));
                let current_y = *player_y + 50.0 / 2.0f32.powf((i + 1) as f32);
                *player_y = current_y;
                rtos::release_mutex(player_y_mut, PLAYER_Y_MUTEX.release(player_y));

                let rect = Rectangle::new(
                    Point::new(PLAYER_X, LINE_Y - current_y as i32 - DINO_SIZE.1 as i32 - 1),
                    Size::new(DINO_SIZE.0, DINO_SIZE.1),
                );

                let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
                rect.draw_styled(&draw, screen).unwrap();
                rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));

                rtos::sleep_ms(150);

                let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
                rect.draw_styled(&ERASE, screen).unwrap();
                rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));
            }

            50.0
        } else if current_y > 0.0 {
            let player_y = PLAYER_Y_MUTEX.get(rtos::take_mutex(player_y_mut));
            let next_y = current_y - FALL_SPEED * 0.1;
            *player_y = next_y;
            rtos::release_mutex(player_y_mut, PLAYER_Y_MUTEX.release(player_y));
            next_y
        } else {
            let player_y = PLAYER_Y_MUTEX.get(rtos::take_mutex(player_y_mut));
            *player_y = 0.0;
            rtos::release_mutex(player_y_mut, PLAYER_Y_MUTEX.release(player_y));

            let rect = Rectangle::new(
                Point::new(0, LINE_Y),
                Size::new(240, 1),
            );
            let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
            rect.draw_styled(&PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build(), screen).unwrap();
            rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));

            0.0
        };

        let rect = Rectangle::new(
            Point::new(PLAYER_X, LINE_Y - next_y as i32 - DINO_SIZE.1 as i32),
            Size::new(DINO_SIZE.0, DINO_SIZE.1),
        );

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        rect.draw_styled(&draw, screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));

        rtos::sleep_ms(100);

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        rect.draw_styled(&ERASE, screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));
    }
}

#[entry]
fn main() -> ! {
    initialize(
        |rtos| {
            rtos.add_thread(&byte_str("input_direction"), 1, input_direction)
                .unwrap();
            rtos.add_thread(&byte_str("restart_game"), 4, restart_game)
                .unwrap();

            // Cactus Generator is TID = 2

            let player_y_mut = rtos.init_mutex(&PLAYER_Y_MUTEX).unwrap();
            let player_speed_mut = rtos.init_mutex(&PLAYER_SPEED_MUTEX).unwrap();
            let direction_mut = rtos.init_mutex(&DIRECTION_MUTEX).unwrap();

            let restart_sem = rtos.init_semaphore(0).unwrap();
            let score_sem = rtos.init_semaphore(0).unwrap();

            unsafe {
                PLAYER_Y_MUT.set(player_y_mut);
                PLAYER_SPEED_MUT.set(player_speed_mut);
                DIRECTION_MUT.set(direction_mut);

                PLAYER_Y_MUTEX.init(0.0);
                PLAYER_SPEED_MUTEX.init(10.0);
                DIRECTION_MUTEX.init(None);

                RESTART_SEM.set(restart_sem);
                SCORE_SEM.set(score_sem);
            }
        },
        Some(|| {
            rtos::signal_semaphore(&*RESTART_SEM);
        }),
        None,
        None,
        None,
        None,
    );
}
