#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::f32::consts::PI;
use micromath::F32Ext as _;

use cortex_m_rt::entry;
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyleBuilder, Rectangle, StyledDrawable as _},
};
use panic_halt as _;

use eel4745c::{
    byte_str,
    game::*,
    physics::{CollisionShape, GameObject},
    rtos::{self, G8torMutex, G8torMutexHandle, G8torThreadHandle},
    SyncUnsafeOnceCell,
};
use rand::{Rng, SeedableRng};

// Constants
const GAME_BOUNDS: (i32, i32, i32, i32) = (10, 230, 50, 270); // (x_min, x_max, y_min, y_max)
const BOUNDS_RECT: Rectangle = Rectangle::new(
    Point::new(GAME_BOUNDS.0 - 1, GAME_BOUNDS.2 - 1),
    Size::new(
        (GAME_BOUNDS.1 - GAME_BOUNDS.0) as u32 + 2,
        (GAME_BOUNDS.3 - GAME_BOUNDS.2) as u32 + 2,
    ),
);

const PLAYER_WIDTH: u32 = 30; // px
const PLAYER_HEIGHT: u32 = 2; // px
const PLAYER_SPEED: f32 = 20.0; // px/s
const BALL_SPEED: f32 = 10.0; // px/s
const BALL_RADIUS: u32 = 5; // px

// Atomics

// Shared Resources
static PLAYER_POS_MUTEX: G8torMutex<f32> = G8torMutex::empty();

// Mutex Handles
static PLAYER_POS_MUT: SyncUnsafeOnceCell<G8torMutexHandle<f32>> = SyncUnsafeOnceCell::new();

// FIFOs

// Semaphore Handles

// Threads
extern "C" fn input_movement(_rtos: G8torThreadHandle) -> ! {
    let player_pos_mut = &*PLAYER_POS_MUT;
    loop {
        let (x, _) = get_joystick();

        let player_pos = PLAYER_POS_MUTEX.get(rtos::take_mutex(player_pos_mut));
        *player_pos += PLAYER_SPEED * 0.1 * x;
        *player_pos = (*player_pos).clamp(
            (GAME_BOUNDS.0 + (PLAYER_WIDTH / 2) as i32) as f32,
            (GAME_BOUNDS.1 - (PLAYER_WIDTH / 2) as i32) as f32,
        );
        rtos::release_mutex(player_pos_mut, PLAYER_POS_MUTEX.release(player_pos));
    }
}

extern "C" fn player_thread(_rtos: G8torThreadHandle) -> ! {
    let player_pos_mut = &*PLAYER_POS_MUT;
    let screen_mut = &*SCREEN_MUT;

    let draw = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::WHITE)
        .stroke_width(0)
        .build();
    let erase = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .stroke_width(0)
        .build();

    loop {
        // Get current posititon
        let player_pos = PLAYER_POS_MUTEX.get(rtos::take_mutex(player_pos_mut));
        let current_pos = *player_pos;
        rtos::release_mutex(player_pos_mut, PLAYER_POS_MUTEX.release(player_pos));

        // Draw two rectangles (one on upper bound and one on lower)
        let current_pos = current_pos as i32;
        let upper_rect = Rectangle::new(
            Point::new(current_pos - (PLAYER_WIDTH / 2) as i32, GAME_BOUNDS.2),
            Size::new(PLAYER_WIDTH, PLAYER_HEIGHT),
        );
        let lower_rect = Rectangle::new(
            Point::new(
                current_pos - (PLAYER_WIDTH / 2) as i32,
                GAME_BOUNDS.3 - PLAYER_HEIGHT as i32,
            ),
            Size::new(PLAYER_WIDTH, PLAYER_HEIGHT),
        );

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        upper_rect.draw_styled(&draw, screen).unwrap();
        lower_rect.draw_styled(&draw, screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));

        rtos::sleep_ms(50);

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(screen_mut));
        upper_rect.draw_styled(&erase, screen).unwrap();
        lower_rect.draw_styled(&erase, screen).unwrap();
        rtos::release_mutex(screen_mut, SCREEN_MUTEX.release(screen));
    }
}

extern "C" fn ball_thread(rtos: G8torThreadHandle) -> ! {
    let player_pos_mut = &*PLAYER_POS_MUT;
    let draw = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::WHITE)
        .stroke_width(0)
        .build();
    let erase = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .stroke_width(0)
        .build();

    let mut rand = rand::rngs::SmallRng::seed_from_u64(rtos::get_system_time() as u64);

    let mut ball_object: GameObject<f32> = GameObject {
        position: (
            (GAME_BOUNDS.1 + GAME_BOUNDS.0) as f32 / 2.0,
            (GAME_BOUNDS.3 + GAME_BOUNDS.2) as f32 / 2.0,
        ),
        collider: CollisionShape::Circle {
            radius: BALL_RADIUS as f32,
        },
    };

    let dir = rand.random_range((-0.85 * PI)..(-0.15 * PI));
    let mut vel = (BALL_SPEED * dir.cos(), BALL_SPEED * dir.sin());

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
        let player_pos_ref = PLAYER_POS_MUTEX.get(rtos::take_mutex(player_pos_mut));
        let player_pos = *player_pos_ref;
        rtos::release_mutex(player_pos_mut, PLAYER_POS_MUTEX.release(player_pos_ref));

        let upper_player_object = GameObject {
            position: (
                player_pos,
                (GAME_BOUNDS.2 + (PLAYER_HEIGHT / 2) as i32) as f32,
            ),
            collider: CollisionShape::Rectangle {
                width: PLAYER_WIDTH as f32,
                height: PLAYER_HEIGHT as f32,
            },
        };
        let lower_player_object = GameObject {
            position: (
                player_pos,
                (GAME_BOUNDS.3 - (PLAYER_HEIGHT / 2) as i32) as f32,
            ),
            collider: CollisionShape::Rectangle {
                width: PLAYER_WIDTH as f32,
                height: PLAYER_HEIGHT as f32,
            },
        };

        if (vel.1 < 0.0 && ball_object.collides_with(&upper_player_object))
            || (vel.1 > 0.0 && ball_object.collides_with(&lower_player_object))
        {
            // Snap to edge of player
            if vel.1 < 0.0 {
                ball_object.position.1 = upper_player_object.position.1
                    + (PLAYER_HEIGHT as f32 / 2.0)
                    + (BALL_RADIUS as f32);
            } else {
                ball_object.position.1 = lower_player_object.position.1
                    - (PLAYER_HEIGHT as f32 / 2.0)
                    - (BALL_RADIUS as f32);
            }

            // Bounce off player
            vel.1 *= -1.0;

            // Randomly jitter velocity and increase speed
            let jitter: (f32, f32) = (rand.random_range(-1.0..1.0), rand.random_range(-1.0..1.0));
            vel.0 = (vel.0 + jitter.0) * 1.1;
            vel.1 = (vel.1 + jitter.1) * 1.1;
        }

        if ball_object.position.0 < (GAME_BOUNDS.0 + BALL_RADIUS as i32) as f32
            || ball_object.position.0 > (GAME_BOUNDS.1 - BALL_RADIUS as i32) as f32
        {
            // Snap to edge of wall
            if vel.0 < 0.0 {
                ball_object.position.0 = (GAME_BOUNDS.0 + BALL_RADIUS as i32) as f32;
            } else {
                ball_object.position.0 = (GAME_BOUNDS.1 - BALL_RADIUS as i32) as f32;
            }

            // Bounce off wall
            vel.0 *= -1.0;
        }

        if ball_object.position.1 < GAME_BOUNDS.2 as f32
            || ball_object.position.1 > GAME_BOUNDS.3 as f32
        {
            rtos::spawn_thread(&byte_str("ball"), 1, ball_thread);
            rtos.kill();
        }

        // Update position
        ball_object.position.0 += vel.0 * 0.1;
        ball_object.position.1 += vel.1 * 0.1;

        // Draw ball
        let ball_circle = Circle::new(
            Point::new(
                ball_object.position.0 as i32 - BALL_RADIUS as i32 + 1,
                ball_object.position.1 as i32 - BALL_RADIUS as i32 + 1,
            ),
            BALL_RADIUS * 2 - 2,
        );

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(&*SCREEN_MUT));
        ball_circle.draw_styled(&draw, screen).unwrap();
        rtos::release_mutex(&*SCREEN_MUT, SCREEN_MUTEX.release(screen));

        rtos::sleep_ms(100);

        let screen = SCREEN_MUTEX.get(rtos::take_mutex(&*SCREEN_MUT));
        ball_circle.draw_styled(&erase, screen).unwrap();
        rtos::release_mutex(&*SCREEN_MUT, SCREEN_MUTEX.release(screen));
    }
}

#[entry]
fn main() -> ! {
    initialize(
        |rtos| {
            rtos.add_thread(&byte_str("player"), 1, player_thread)
                .unwrap();
            rtos.add_thread(&byte_str("input_direction"), 1, input_movement)
                .unwrap();
            rtos.add_thread(&byte_str("ball"), 1, ball_thread).unwrap();

            let player_pos_mut = rtos.init_mutex(&PLAYER_POS_MUTEX).unwrap();

            unsafe {
                PLAYER_POS_MUTEX.init((GAME_BOUNDS.1 - GAME_BOUNDS.0) as f32 / 2.0);
                PLAYER_POS_MUT.set(player_pos_mut);
            }
        },
        None,
        None,
        None,
        None,
        None,
    );
}
