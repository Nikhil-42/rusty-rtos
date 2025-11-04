#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::sync::atomic::AtomicBool;

use cortex_m_rt::entry;
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle, StyledDrawable as _},
};
use panic_halt as _;

use eel4745c::{
    SyncUnsafeOnceCell, byte_str, game::*, physics::Direction, rtos::{self, G8torMutex, G8torMutexHandle, G8torSemaphoreHandle, G8torThreadHandle}
};
use rand::{Rng, SeedableRng};

// Constants
const SCALE: u32 = 10;
const SCALE_SIZE: Size = Size::new(SCALE, SCALE);
const GAME_BOUNDS: (i32, i32, i32, i32) = (1, 22, 5, 26); // (x_min, x_max, y_min, y_max)
const BOUNDS_RECT: Rectangle = Rectangle::new(
    Point::new(GAME_BOUNDS.0 * SCALE as i32 - 1, GAME_BOUNDS.2 * SCALE as i32 - 1),
    Size::new((GAME_BOUNDS.1 - GAME_BOUNDS.0 + 1) as u32 * SCALE + 2, (GAME_BOUNDS.3 - GAME_BOUNDS.2 + 1) as u32 * SCALE + 2),
);

const MAX_SNAKE_LENGTH: usize = 64;

// Atomics
static DIE: AtomicBool = AtomicBool::new(false);

// Shared Resources
static DIRECTION_MUTEX: G8torMutex<Direction> = G8torMutex::empty();

// Mutex Handles
static DIRECTION_MUT: SyncUnsafeOnceCell<G8torMutexHandle<Direction>> = SyncUnsafeOnceCell::new();

// FIFOs

// Semaphore Handles
static RESTART_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();
static SCORE_SEM: SyncUnsafeOnceCell<G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();

// Threads
extern "C" fn input_direction(_rtos: G8torThreadHandle) -> ! {
    loop {
        let (x, y) = get_joystick();

        let direction = DIRECTION_MUTEX.get(rtos::take_mutex(&*DIRECTION_MUT));
        if y.abs() < 0.5 {
            if x > 0.5 {
                *direction = Direction::Right;
            } else if x < -0.5 {
                *direction = Direction::Left;
            }
        } else if x.abs() < 0.5 {
            if y > 0.5 {
                *direction = Direction::Up;
            } else if y < -0.5 {
                *direction = Direction::Down;
            }
        }
        rtos::release_mutex(&*DIRECTION_MUT, DIRECTION_MUTEX.release(direction));
    }
}

extern "C" fn snake_thread(rtos: G8torThreadHandle) -> ! {
    let direction = DIRECTION_MUTEX.get(rtos::take_mutex(&*DIRECTION_MUT));
    *direction = Direction::Right;
    rtos::release_mutex(&*&DIRECTION_MUT, DIRECTION_MUTEX.release(direction));

    let mut chain = [Direction::Right; MAX_SNAKE_LENGTH];
    let mut length = 4;
    let mut head_ptr: usize = length - 1;
    let mut tail_ptr: usize = 0;

    let mut pos: (i32, i32) = (GAME_BOUNDS.0, (GAME_BOUNDS.2 + GAME_BOUNDS.3) / 2);
    let mut tail_pos = (pos.0 - (length as i32 - 1), pos.1);

    let mut rand = rand::rngs::SmallRng::seed_from_u64(rtos::get_system_time() as u64);
    let mut fruit_pos = (
        rand.random_range(GAME_BOUNDS.0..GAME_BOUNDS.1),
        rand.random_range(GAME_BOUNDS.2..GAME_BOUNDS.3),
    );

    let erase = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();
    let draw = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::GREEN)
        .build();
    let fruit_style = PrimitiveStyleBuilder::new().fill_color(Rgb565::RED).build();
    let bounds_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::WHITE)
        .stroke_width(1)
        .build();

    // Draw initial frame
    let fruit_square = Rectangle::new(
        Point::new(
            fruit_pos.0 as i32 * SCALE as i32,
            fruit_pos.1 as i32 * SCALE as i32,
        ),
        SCALE_SIZE,
    );
    let bounds_rect = BOUNDS_RECT;

    let screen = SCREEN_MUTEX.get(rtos::take_mutex(&*SCREEN_MUT));
    fruit_square.draw_styled(&fruit_style, screen).unwrap();
    bounds_rect.draw_styled(&bounds_style, screen).unwrap();
    rtos::release_mutex(&*SCREEN_MUT, SCREEN_MUTEX.release(screen));

    loop {
        if DIE.load(core::sync::atomic::Ordering::Relaxed) {
            // Confirm death
            rtos::signal_semaphore(&*&RESTART_SEM);
            rtos.kill();
        }

        let direction = DIRECTION_MUTEX.get(rtos::take_mutex(&*DIRECTION_MUT));
        let dir = *direction;
        rtos::release_mutex(&*DIRECTION_MUT, DIRECTION_MUTEX.release(direction));

        // Prevent turn-in-place
        let last_dir = chain[if head_ptr > 0 {head_ptr - 1} else {MAX_SNAKE_LENGTH - 1}];
        let dir = if dir != last_dir.opposite() { dir } else { last_dir };

        // Calculate next position
        let next_pos = dir.push(pos);
        let next_tail_pos = chain[tail_ptr].push(tail_pos);
        chain[head_ptr] = dir;

        let head_square = Rectangle::new(
            Point::new(next_pos.0 * SCALE as i32, next_pos.1 * SCALE as i32),
            SCALE_SIZE,
        );

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(&*SCREEN_MUT));
        head_square.draw_styled(&draw, screen).unwrap();
        if next_pos.0 < GAME_BOUNDS.0 as i32
            || next_pos.0 > GAME_BOUNDS.1 as i32
            || next_pos.1 < GAME_BOUNDS.2 as i32
            || next_pos.1 > GAME_BOUNDS.3 as i32
        {
            // Out of bounds
            rtos::release_mutex(&*SCREEN_MUT, SCREEN_MUTEX.release(screen));
            // Trigger restart
            rtos::signal_semaphore(&*RESTART_SEM);
            // Confirm death
            rtos::signal_semaphore(&*RESTART_SEM);
            rtos.kill();
        } else if (0..length)
            .map(|i| chain[(tail_ptr + i) % MAX_SNAKE_LENGTH])
            .scan(tail_pos, |state, d| {
                let current = *state;
                *state = d.push(current);
                Some(current)
            }) // Get all snake positions
            .any(|p| p == next_pos)
        {
            // Collided with self
            rtos::release_mutex(&*SCREEN_MUT, SCREEN_MUTEX.release(screen));
            // Trigger restart
            rtos::signal_semaphore(&*RESTART_SEM);
            // Confirm death
            rtos::signal_semaphore(&*RESTART_SEM);
            rtos.kill();
        } else if next_pos == fruit_pos {
            // Ate fruit
            length += 1;
            rtos::signal_semaphore(&*&SCORE_SEM);

            fruit_pos = (
                rand.random_range(GAME_BOUNDS.0..GAME_BOUNDS.1),
                rand.random_range(GAME_BOUNDS.2..GAME_BOUNDS.3),
            );
            let fruit_square = Rectangle::new(
                Point::new(fruit_pos.0 * SCALE as i32, fruit_pos.1 * SCALE as i32),
                SCALE_SIZE,
            );
            fruit_square.draw_styled(&fruit_style, screen).unwrap();
        } else {
            // Normal movement
            let tail_square = Rectangle::new(
                Point::new(tail_pos.0 * SCALE as i32, tail_pos.1 * SCALE as i32),
                SCALE_SIZE,
            );
            tail_square.draw_styled(&erase, screen).unwrap();
            tail_pos = next_tail_pos;
            tail_ptr = (tail_ptr + 1) % MAX_SNAKE_LENGTH;
        }
        rtos::release_mutex(&*SCREEN_MUT, SCREEN_MUTEX.release(screen));

        head_ptr = (head_ptr + 1) % MAX_SNAKE_LENGTH;
        pos = next_pos;
        rtos::sleep_ms(1000);
    }
}

extern "C" fn restart_game(_rtos: G8torThreadHandle) -> ! {
    loop {
        rtos::wait_semaphore(&*&RESTART_SEM);
        DIE.store(true, core::sync::atomic::Ordering::Relaxed);
        rtos::wait_semaphore(&*&RESTART_SEM);
        DIE.store(false, core::sync::atomic::Ordering::Relaxed);

        rtos::sleep_ms(3000);

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(&*SCREEN_MUT));
        screen.clear(Rgb565::BLACK).unwrap();
        rtos::release_mutex(&*SCREEN_MUT, SCREEN_MUTEX.release(screen));

        rtos::spawn_thread(&byte_str("snake"), 1, snake_thread);
    }
}


#[entry]
fn main() -> ! {
    unsafe {
        DIRECTION_MUTEX.init(Direction::Right);
    }
    initialize(|rtos| {
        rtos.add_thread(&byte_str("snake"), 1, snake_thread).unwrap();
        rtos.add_thread(&byte_str("restart"), 1, restart_game).unwrap();
        rtos.add_thread(&byte_str("input_direction"), 1, input_direction).unwrap();

        let direction_mut = rtos.init_mutex(&DIRECTION_MUTEX).unwrap();
        let restart_sem = rtos.init_semaphore(0).unwrap();
        let score_sem = rtos.init_semaphore(0).unwrap();

        unsafe {
            DIRECTION_MUT.set(direction_mut);
            RESTART_SEM.set(restart_sem);
            SCORE_SEM.set(score_sem);
        }
    }, 
    None, None, None, None, Some(|| {
        rtos::signal_semaphore(&*RESTART_SEM);
    }));
}
