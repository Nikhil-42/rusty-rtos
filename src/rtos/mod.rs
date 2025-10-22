mod atomics;
mod handlers;

use crate::rtos::atomics::{
    G8torAtomic, G8torAtomicHandle, G8torMutexHandle, G8torSemaphoreHandle,
};
use crate::rtos::handlers::SyscallReturn;
use crate::syscall;

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::mem::{ManuallyDrop, MaybeUninit};
use core::num::NonZeroU8;
use core::sync::{atomic::AtomicBool};
use core::{arch::asm, ptr::NonNull};

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use tm4c123x_hal::pac::adc0::dccmp3::W;

const MAX_THREADS: usize = 6;
const STACK_SIZE: usize = 512; // 2KB stack
const NUM_ATOMICS: usize = 8;
const NAME_LEN: usize = 16;
const PERIOD_US: u32 = 1_000_000; // SysTick period in microseconds
const TICKS_PER_US: u32 = 16; // Assuming the 16 MHz core clock

// SAFETY: It is never safe to have any references to this static while the RTOS is running because system_time is volatile
static mut G8TOR_RTOS: MaybeUninit<G8torRtos> = MaybeUninit::uninit();

#[repr(C)]
struct TCB {
    pub id: usize,
    pub sp: NonNull<u32>, // Stack pointer (points to top of stack)
    pub next: NonNull<TCB>,
    pub prev: NonNull<TCB>,
    pub sleep_until: u32,
    pub asleep: bool,
    pub priority: u8,
    pub blocked_by: Option<G8torAtomicHandle>, // A handle to the blocking atomic (if any)
    pub name: [u8; NAME_LEN],
}

#[repr(C)]
pub struct G8torRtos {
    running: Option<NonNull<TCB>>,
    system_time: u32,
    threads: [Option<TCB>; MAX_THREADS],
    atomics: [G8torAtomic; NUM_ATOMICS],
    atomic_mask: u8,
    stacks: MaybeUninit<[[u32; STACK_SIZE]; MAX_THREADS]>,
    peripherals: cortex_m::Peripherals,
}

// Executes in critical section so we have exclusive access to G8TOR_RTOS
unsafe extern "C" fn _scheduler(rtos: &mut G8torRtos) -> Option<NonNull<TCB>> {
    let mut next_thread = if let Some(running) = rtos.running.as_mut() {
        // If there's a running thread, start searching from its next thread
        running.as_mut().next.as_mut()
    } else {
        // No running thread, start from the first living thread
        let mut next_thread = None;
        for t in rtos.threads.iter_mut() {
            if let Some(tcb) = t.as_mut() {
                next_thread = Some(tcb);
                break;
            }
        }
        next_thread?
    };

    let start = next_thread.prev.as_ref().id;  // To detect when we've looped through all threads
    let mut searching = true;
    while searching {
        if next_thread.id == start {
            // We have looped through all threads if we can't 
            // run this one then there are no runnable threads
            searching = false;
        }

        if next_thread.asleep {
            // Check if the deadline has passed
            let current_time = rtos.system_time;
            if next_thread.sleep_until.wrapping_sub(current_time) as i32 <= 0 {
                // Wake up the thread
                next_thread.asleep = false;
            } else {
                // Move to the next thread
                next_thread = next_thread.next.as_mut();
                continue;
            }
        }
        // next_thread is not asleep

        if next_thread.blocked_by.is_some() {
            // Thread is blocked on an atomic, move to the next thread
            next_thread = next_thread.next.as_mut();
            continue;
        }
        // next_thread is not blocked

        return Some(NonNull::new_unchecked(next_thread as *mut TCB));
    };
    
    // No thread is ready to run, return None
    None
}

unsafe extern "C" fn _syscall(r0: usize, r1: usize, r2: usize, r3: usize, imm: u8) -> usize {
    match imm {
        0 => {
            // Pend a context switch 
            cortex_m::peripheral::SCB::set_pendsv();

            if r0 > 0 {
                // Sleep syscall
                let rtos = &raw const G8TOR_RTOS as *const G8torRtos;
                let system_time = &raw const (*rtos).system_time;
                let sleep_time = r0 as u32;
                let current_time = core::ptr::read_volatile(system_time);
                let wake_time = current_time.wrapping_add(sleep_time);

                // SAFTEY: This is safe because PendSV will not run until after we exit the syscall
                // so the running thread cannot change while we are in this function
                let running_tcb = (*rtos).running.unwrap_unchecked().as_ptr();
                (*running_tcb).asleep = true;
                (*running_tcb).sleep_until = wake_time;
            }

            0
        },
        _ => 0, // Unknown syscall
    }
}

impl G8torRtos {
    #[allow(static_mut_refs)]
    pub unsafe fn new(peripherals: cortex_m::Peripherals) -> &'static mut Self {
        // Disable interrupts during initialization
        cortex_m::interrupt::disable();

        // SAFETY: We only write to G8TOR_RTOS here no reads
        // We are in a single-threaded context (no interrupts)
        // so this is the only access to G8TOR_RTOS
        let ptr = G8TOR_RTOS.as_mut_ptr();
        (&raw mut (*ptr).running).write(None);
        (&raw mut (*ptr).system_time).write(0);
        (&raw mut (*ptr).threads).write([const { None }; MAX_THREADS]);
        (&raw mut (*ptr).atomics).write([const { G8torAtomic::new_semaphore(0u8) }; NUM_ATOMICS]);
        (&raw mut (*ptr).stacks).write(MaybeUninit::uninit());
        (&raw mut (*ptr).peripherals).write(peripherals);
        // G8TOR_RTOS is now fully initialized
        // Except running which will be set when launching

        // Return a mutable reference to the static instance
        // To enable user code to add threads
        G8TOR_RTOS.assume_init_mut()
    }

    /// Add a thread to the RTOS
    /// Safety: This function should only be called, at the start of the program BEFORE launch
    pub fn add_thread(&mut self, name: &[u8; NAME_LEN], thread: extern "C" fn(G8torRtosHandle) -> !) -> Result<(), ()> {
        // Find an empty TCB slot
        let _self_ptr = self as *mut Self;
        for id in 0..MAX_THREADS {
            if self.threads[id].is_none() {
                // Initialize the stack for the new thread
                // This is okay because self.stacks is a MaybeUninit and we only write to it here
                let stack = self.stacks.as_mut_ptr().cast::<[u32; STACK_SIZE]>();
                let stack = unsafe { stack.add(id) }; // SAFETY: id < MAX_THREADS so this is in bounds
                let sp = unsafe { stack.cast::<u32>().add(STACK_SIZE) }; // SAFETY: stack is valid and STACK_SIZE is in bounds

                // Reserve const space for the initial stack frame
                let sp = unsafe { sp.sub(16) }; // SAFETY: STACK_SIZE must be > 64 bytes

                // Set up initial stack frame
                // SAFETY: We just reserved space for 16 u32s and sp is valid
                unsafe {
                    sp.add(15).write(0x01000000); // xPSR
                    sp.add(14).write(thread as u32); // PC
                    sp.add(13).write(0x14141414); // R14 (LR)
                    sp.add(12).write(0x12121212); // R12
                    sp.add(11).write(0x03030303); // R3
                    sp.add(10).write(0x02020202); // R2
                    sp.add(9).write(0x01010101); // R1
                    sp.add(8).write(id as u32); // R0
                    sp.add(7).write(0x11111111); // R11
                    sp.add(6).write(0x10101010); // R10
                    sp.add(5).write(0x09090909); // R9
                    sp.add(4).write(0x08080808); // R8
                    sp.add(3).write(0x07070707); // R7
                    sp.add(2).write(0x06060606); // R6
                    sp.add(1).write(0x05050505); // R5
                    sp.add(0).write(0x04040404); // R4
                }

                // Create the new TCB and link it into the circular doubly linked list
                let thread = self.threads[id].insert(TCB {
                    id: id,
                    sp: NonNull::new(sp as *mut u32)
                        .expect("Obviously sp is not null because we just wrote to it."),
                    next: NonNull::dangling(),
                    prev: NonNull::dangling(),
                    sleep_until: 0,
                    asleep: false,
                    priority: 0,
                    blocked_by: None,
                    name: *name,
                });

                if id > 0 {
                    thread.next = unsafe {
                        NonNull::new_unchecked((*_self_ptr).threads[0].as_mut().unwrap_unchecked())
                    }; // SAFETY: NUM_THREADS > 0 and id = i implies that threads[k] is Some for k < i
                       // so k = 0 is definitely Some
                    thread.prev = unsafe {
                        NonNull::new_unchecked(
                            (*_self_ptr).threads[id - 1].as_mut().unwrap_unchecked(),
                        )
                    }; // SAFETY: We just checked that i > 0 also id = i implies that threads[k] is Some for k < i
                       // so k = i-1 is definitely Some

                    // Update the previous TCB's next pointer to point to the new TCB
                    unsafe {
                        self.threads[id - 1].as_mut().unwrap_unchecked().next =
                            NonNull::new_unchecked(thread as *mut TCB)
                    }; // SAFETY: We just checked that i > 0 also id = i implies that threads[k] is Some for k < i
                } else {
                    // First thread, points to itself
                    // SAFETY: We just initialized thread so it is definitely Some
                    thread.next = unsafe { NonNull::new_unchecked(thread as *mut TCB) };
                    thread.prev = unsafe { NonNull::new_unchecked(thread as *mut TCB) };
                }

                return Ok(());
            }
        }
        return Err(()); // No empty slot found
    }

    pub fn init_semaphore(
        &mut self,
        initial_count: u8,
        max_count: u8,
    ) -> Result<G8torSemaphoreHandle, ()> {
        for index in 0..NUM_ATOMICS {
            if (self.atomic_mask & (1 << index)) == 0 {
                // Found an empty atomic slot
                self.atomic_mask |= 1 << index;
                self.atomics[index] = G8torAtomic::new_semaphore(initial_count.min(max_count));
                // SAFETY: index < NUM_ATOMICS < 256 so index + 1 < 256 so this is a valid NonZeroU8
                return Ok(G8torSemaphoreHandle { index: unsafe { NonZeroU8::new_unchecked((index + 1) as u8) }, max_count });
            }
        }

        // No empty atomic slot found
        Err(())
    }

    pub fn init_mutex<'a, T>(&mut self, resource: &'a mut UnsafeCell<T>) -> Result<G8torMutexHandle<'a, T>, ()> {
        for index in 0..NUM_ATOMICS {
            if (self.atomic_mask & (1 << index)) == 0 {
                // Found an empty atomic slot
                self.atomic_mask |= 1 << index;
                self.atomics[index].mutex = ManuallyDrop::new(AtomicBool::new(true));
                // SAFETY: index < NUM_ATOMICS < 256 so index + 1 < 256 so this is a valid NonZeroU8
                return Ok(G8torMutexHandle { index: unsafe { NonZeroU8::new_unchecked((index + 1) as u8) }, resource: resource });
            }
        }

        // No empty atomic slot found
        Err(())
    }

    pub unsafe fn launch(&mut self) -> ! {
        let _self_ptr = self as *mut Self;

        // Set the currently running thread to the first thread added
        self.running = match (*_self_ptr).threads[0].as_mut() {
            Some(tcb) => Some(NonNull::new_unchecked(tcb as *mut TCB)),
            None => panic!("No threads to run!"),
        };

        // Start SysTick
        let syst = &mut self.peripherals.SYST;
        let scb = &mut self.peripherals.SCB;
        syst.disable_counter();

        syst.clear_current();
        syst.set_clock_source(SystClkSource::Core);  // External maps to the PIOSC / 4 (4 MHz)
                                                                // Core maps to the system clock (16 MHz in our case)
        syst.set_reload(PERIOD_US * TICKS_PER_US - 1); // 1 ms
        unsafe {
            scb.set_priority(SystemHandler::SysTick, 0); // Highest priority
            scb.set_priority(SystemHandler::PendSV, 0b11100000); // Lowest priority
        };
        syst.enable_interrupt(); // Note: interrupts are still disabled globally
        syst.enable_counter();

        // Start the first thread by loading its context
        asm!(
            "ldr sp, [{sp}, #4]",     // u32* r0 = self.running->sp
            "pop {{r4-r11}}",         // Restore callee-saved registers
            "pop {{r0-r3, r12}}",     // Restore caller-saved registers
            "add sp, sp, #4",         // Skip LR
            "pop {{lr}}",             // Restore pc into lr
            "add sp, sp, #4",         // Skip xPSR
            "cpsie i",                // Enable interrupts
            "bx lr",                  // Branch to the thread's PC
            sp = in(reg) self.running.unwrap_unchecked().as_ptr(),
            options(noreturn)
        )
    }
}

#[repr(C)]
pub struct G8torRtosHandle {
    id: usize,
    _rtos: PhantomData<&'static G8torRtos>,
}

impl G8torRtosHandle {
    pub fn yield_now(&self) {
        // Sleep for 0 ticks to yield the CPU
        syscall!(0; 0);
    }

    pub fn sleep_ms(&self, ms: usize) {
        syscall!(0; ms);
    }
}