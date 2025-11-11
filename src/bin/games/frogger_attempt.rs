#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use cortex_m_rt::entry;
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle, StyledDrawable as _},
};
use panic_halt as _;

use eel4745c::{
    byte_str,
    game::*,
    physics::Direction,
    rtos::{self, G8torMutex, G8torMutexHandle, G8torThreadHandle},
    SyncUnsafeOnceCell,
};

// Constants
// if next_pos.0 < GAME_BOUNDS.0 as i32
//             || next_pos.0 > GAME_BOUNDS.1 as i32
//             || next_pos.1 < GAME_BOUNDS.2 as i32
//             || next_pos.1 > GAME_BOUNDS.3 as i32

const SCALE: u32 = 20;
const SCALE_SIZE: Size = Size::new(SCALE, SCALE);
const GAME_BOUNDS: (i32, i32, i32, i32) = (2, 9, 2, 13); // (x_min, x_max, y_min, y_max)
const BOUNDS_RECT: Rectangle = Rectangle::new(
    Point::new(
        GAME_BOUNDS.0 * SCALE as i32 - 1,
        GAME_BOUNDS.2 * SCALE as i32 - 1,
    ),
    Size::new(
        (GAME_BOUNDS.1 - GAME_BOUNDS.0 + 1) as u32 * SCALE + 2,
        (GAME_BOUNDS.3 - GAME_BOUNDS.2 + 1) as u32 * SCALE + 2,
    ),
);

// const NUM_COLS: usize = (GAME_BOUNDS.1 - GAME_BOUNDS.0 + 1) as usize;

// Atomics
// Shared Resources
static PLAYER_POS_MUTEX: G8torMutex<(i32, i32)> = G8torMutex::empty();
static DIRECTION_MUTEX: G8torMutex<Option<Direction>> = G8torMutex::empty();

// Mutex Handles
static PLAYER_POS_MUT: SyncUnsafeOnceCell<G8torMutexHandle<(i32, i32)>> = SyncUnsafeOnceCell::new();
static DIRECTION_MUT: SyncUnsafeOnceCell<G8torMutexHandle<Option<Direction>>> =
    SyncUnsafeOnceCell::new();

// FIFOs
static ROW_CONFIG_FIFO: SyncUnsafeOnceCell<rtos::G8torFifoHandle> = SyncUnsafeOnceCell::new();

// Semaphore Handles
static ROW_READY_SEM: SyncUnsafeOnceCell<rtos::G8torSemaphoreHandle> = SyncUnsafeOnceCell::new();

// Threads
extern "C" fn input_movement(_rtos: G8torThreadHandle) -> ! {
    let direction_mut = &*DIRECTION_MUT;
    loop {
        let (x, y) = get_joystick();

        let direction = DIRECTION_MUTEX.get(rtos::take_mutex(direction_mut));
        if y.abs() < 0.5 && x.abs() >= 0.5 {
            // Move side to side
            if x > 0.0 {
                // Right
                *direction = Some(Direction::Right);
            } else {
                // Left
                *direction = Some(Direction::Left);
            }
        } else if x.abs() < 0.5 && y.abs() >= 0.5 {
            // Move up and down
            if y > 0.0 {
                // Up
                *direction = Some(Direction::Up);
            } else {
                // Down
                *direction = Some(Direction::Down);
            }
        } else {
            *direction = None;
        }

        rtos::release_mutex(direction_mut, DIRECTION_MUTEX.release(direction));
    }
}

extern "C" fn player_thread(_rtos: G8torThreadHandle) -> ! {
    let player_pos_mut = &*PLAYER_POS_MUT;
    let screen_mut = &*SCREEN_MUT;

    let draw = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::GREEN)
        .stroke_width(0)
        .build();
    let erase = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .stroke_width(0)
        .build();

    // Draw bounds
    let screen = SCREEN_MUTEX.get(rtos::take_mutex(&*SCREEN_MUT));
    BOUNDS_RECT
        .draw_styled(
            &PrimitiveStyleBuilder::new()
                .stroke_color(Rgb565::WHITE)
                .stroke_width(1)
                .build(),
            screen,
        )
        .unwrap();
    rtos::release_mutex(&*SCREEN_MUT, SCREEN_MUTEX.release(screen));

    loop {
        // Get current posititon
        let player_pos = PLAYER_POS_MUTEX.get(rtos::take_mutex(player_pos_mut));
        let current_pos = *player_pos;
        rtos::release_mutex(player_pos_mut, PLAYER_POS_MUTEX.release(player_pos));

        let direction = DIRECTION_MUTEX.get(rtos::take_mutex(&*DIRECTION_MUT));
        let next_pos = if let Some(direction) = *direction {
            direction.push(current_pos)
        } else {
            current_pos
        };
        rtos::release_mutex(&*DIRECTION_MUT, DIRECTION_MUTEX.release(direction));

        // Draw two rectangles (one on upper bound and one on lower)
        let frog = Rectangle::new(
            Point::new(
                GAME_BOUNDS.0 + current_pos.0 * SCALE as i32,
                GAME_BOUNDS.2 + current_pos.1 * SCALE as i32,
            ),
            SCALE_SIZE,
        );

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        frog.draw_styled(&draw, screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));

        let player_pos = PLAYER_POS_MUTEX.get(rtos::take_mutex(player_pos_mut));
        *player_pos = next_pos;
        rtos::release_mutex(player_pos_mut, PLAYER_POS_MUTEX.release(player_pos));

        // Keep frog on screen for 1 second
        for _ in 0..9 {
            rtos::sleep_ms(100);
            let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
            frog.draw_styled(&draw, screen).unwrap();
            rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));
        }

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        frog.draw_styled(&erase, screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));
    }
}

extern "C" fn row_thread(rtos: G8torThreadHandle) -> ! {
    let player_pos_mut = &*PLAYER_POS_MUT;
    let row_config_fifo = &*ROW_CONFIG_FIFO;
    let row_ready_sem = &*ROW_READY_SEM;

    let speed: f32 = f32::from_bits(rtos::read_fifo(row_config_fifo));
    let row_y: i32 = rtos::read_fifo(row_config_fifo) as i32;
    let occupancy = rtos::read_fifo(row_config_fifo) as u16; // Bitfield of occupied positions
    let fgbg = rtos::read_fifo(row_config_fifo); // Foreground/background colors
    let fg_color: Rgb565 = unsafe { core::mem::transmute((fgbg & 0xFFFF) as u16) };
    let bg_color: Rgb565 = unsafe { core::mem::transmute(((fgbg >> 16) & 0xFFFF) as u16) };
    rtos::signal_semaphore(row_ready_sem);
    rtos::yield_now();

    let draw = PrimitiveStyleBuilder::new()
        .fill_color(fg_color)
        .stroke_width(0)
        .build();
    let erase = PrimitiveStyleBuilder::new()
        .fill_color(bg_color)
        .stroke_width(0)
        .build();

    let mut x = 0.0;
    // let mut rand = rand::rngs::SmallRng::seed_from_u64(rtos::get_system_time() as u64);

    loop {
        let player_pos_ref = PLAYER_POS_MUTEX.get(rtos::take_mutex(player_pos_mut));
        let player_pos = *player_pos_ref;
        rtos::release_mutex(player_pos_mut, PLAYER_POS_MUTEX.release(player_pos_ref));

        if player_pos.1 == row_y {
            // Check for collision with occupied positions
            let player_x_pos = (player_pos.0 as u32) / SCALE;
            if (occupancy & (1 << player_x_pos)) == 0 {
                // Player is on empty space, kill the player
                rtos::spawn_thread(&byte_str("player"), 1, player_thread);
                rtos.kill();
            }
        }

        // Draw row
        let pixel_y = (GAME_BOUNDS.2 + row_y) * SCALE as i32;
        let mut occupancy_bits = occupancy;

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(&*SCREEN_MUT));
        for i in 0..16 {
            let subpixel_x = (i as f32 + x) % 16.0 - 8.0;
            let pixel_x = (subpixel_x * SCALE as f32) as i32
                + (GAME_BOUNDS.0 * SCALE as i32 + GAME_BOUNDS.1 * SCALE as i32) / 2;
            let block = if pixel_x + SCALE as i32 <= GAME_BOUNDS.0 {
                // Completely off left edge
                None
            } else if pixel_x < GAME_BOUNDS.0 * SCALE as i32 && pixel_x + SCALE as i32 > GAME_BOUNDS.0 * SCALE as i32 {
                // Partial block on left edge
                Some(Rectangle::new(
                    Point::new(GAME_BOUNDS.0 * SCALE as i32, pixel_y),
                    Size::new((pixel_x + SCALE as i32 - GAME_BOUNDS.0 * SCALE as i32) as u32, SCALE),
                ))
            } else if pixel_x > GAME_BOUNDS.0 * SCALE as i32 && (pixel_x + SCALE as i32) < GAME_BOUNDS.1 * SCALE as i32 {
                // Full block
                Some(Rectangle::new(
                    Point::new(pixel_x, pixel_y),
                    SCALE_SIZE,
                ))
            } else if pixel_x < GAME_BOUNDS.1 * SCALE as i32 && (pixel_x + SCALE as i32) > GAME_BOUNDS.1 * SCALE as i32 {
                // Partial block on right edge
                Some(Rectangle::new(
                    Point::new(pixel_x, pixel_y),
                    Size::new((GAME_BOUNDS.1 * SCALE as i32 - pixel_x) as u32, SCALE),
                ))
            } else if pixel_x >= GAME_BOUNDS.1 * SCALE as i32 {
                // Completely off right edge
                None
            } else {
                None
            };

            if let Some(block) = block {
                block
                    .draw_styled(
                        if occupancy_bits & 1 == 1 {
                            &draw
                        } else {
                            &erase
                        },
                        screen,
                    )
                    .unwrap();
            }
            occupancy_bits >>= 1;
        }
        rtos::release_mutex(&*SCREEN_MUT, SCREEN_MUTEX.release(screen));

        // Update position
        x += speed;
        x %= 16.0;

        rtos::sleep_ms(100);
    }
}

fn configure_row(speed: f32, row_y: i32, occupancy: u16, fg_color: Rgb565, bg_color: Rgb565) {
    let speed = speed / 10.0; // Slow down speed for visibility
    let row_config_fifo = &*ROW_CONFIG_FIFO;
    let row_ready_sem = &*ROW_READY_SEM;

    let fgbg = ((unsafe { core::mem::transmute::<Rgb565, u16>(bg_color) } as u32) << 16)
        | ((unsafe { core::mem::transmute::<Rgb565, u16>(fg_color) } as u32) << 0);

    rtos::write_fifo(row_config_fifo, speed.to_bits());
    rtos::write_fifo(row_config_fifo, row_y as u32);
    rtos::write_fifo(row_config_fifo, occupancy as u32);
    rtos::write_fifo(row_config_fifo, fgbg);

    rtos::spawn_thread(&byte_str("row"), 1, row_thread);
    rtos::wait_semaphore(row_ready_sem);
}

extern "C" fn init_thread(rtos: G8torThreadHandle) -> ! {
    // Configure rows
    configure_row(1.0, 1, 0x5005, Rgb565::CYAN, Rgb565::BLACK);
    configure_row(-1.0, 2, 0xccc, Rgb565::CYAN, Rgb565::BLACK);
    configure_row(2.0, 3, 0x5005, Rgb565::CYAN, Rgb565::BLACK);
    configure_row(-2.0, 4, 0x0017, Rgb565::CYAN, Rgb565::BLACK);
    configure_row(1.0, 5, 0x7001, Rgb565::CYAN, Rgb565::BLACK);

    rtos.kill();
}
#[entry]
fn main() -> ! {
    initialize(
        |rtos| {
            rtos.add_thread(&byte_str("player"), 1, player_thread)
                .unwrap();
            rtos.add_thread(&byte_str("input_direction"), 1, input_movement)
                .unwrap();
            rtos.add_thread(&byte_str("init"), 0, init_thread).unwrap();

            let player_pos_mut = rtos.init_mutex(&PLAYER_POS_MUTEX).unwrap();
            let direction_mut = rtos.init_mutex(&DIRECTION_MUTEX).unwrap();

            let row_config_fifo = rtos.init_fifo().unwrap();
            let row_ready_sem = rtos.init_semaphore(0).unwrap();

            unsafe {
                PLAYER_POS_MUTEX.init((GAME_BOUNDS.0, (GAME_BOUNDS.3 + GAME_BOUNDS.2) / 2));
                DIRECTION_MUTEX.init(None);

                PLAYER_POS_MUT.set(player_pos_mut);
                DIRECTION_MUT.set(direction_mut);

                ROW_CONFIG_FIFO.set(row_config_fifo);

                ROW_READY_SEM.set(row_ready_sem);
            }
        },
        None,
        None,
        None,
        None,
        None,
    );
}
