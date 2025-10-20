use core::arch::naked_asm;
use core::convert::TryInto as _;
use core::mem::{ManuallyDrop, MaybeUninit};
use core::num::NonZeroU8;
use core::sync::atomic::{AtomicBool, AtomicU8};
use core::{arch::asm, ptr::NonNull};

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{exception, ExceptionFrame};

const MAX_THREADS: usize = 6;
const STACK_SIZE: usize = 512; // 2KB stack
const NUM_ATOMICS: usize = 8;

#[unsafe(naked)]
extern "C" fn syscall<const num: u8>() {
    naked_asm!("svc {num}", num = const num);
}


static mut G8TOR_RTOS: MaybeUninit<G8torRtos> = MaybeUninit::uninit();

#[derive(Clone, Copy)]
pub struct G8torAtomicHandle {
    index: NonZeroU8,
}

impl From<G8torSemaphoreHandle> for G8torAtomicHandle {
    #[inline(always)]
    fn from(sem: G8torSemaphoreHandle) -> Self {
        G8torAtomicHandle { index: sem.index }
    }
}

impl From<G8torMutexHandle> for G8torAtomicHandle {
    #[inline(always)]
    fn from(mutex: G8torMutexHandle) -> Self {
        G8torAtomicHandle { index: mutex.index }
    }
}

#[derive(Clone, Copy)]
pub struct G8torMutexHandle {
    index: NonZeroU8,
}

pub struct G8torMutexLock {}

#[derive(Clone, Copy)]
pub struct G8torSemaphoreHandle {
    index: NonZeroU8,
    max_count: u8,
}

pub union G8torAtomic {
    count: ManuallyDrop<AtomicU8>,
    mutex: ManuallyDrop<AtomicBool>,
}

#[repr(C)]
pub struct TCB {
    pub id: u32,
    pub sp: NonNull<u32>, // Stack pointer (points to top of stack)
    pub next: NonNull<TCB>,
    pub prev: NonNull<TCB>,
    pub sleep_until: u32,
    pub priority: u8,
    pub blocked_by: Option<G8torAtomicHandle>, // A handle to the blocking atomic (if any)
    pub name: [u8; 8],
}

#[repr(C)]
pub struct G8torRtos {
    running: NonNull<TCB>,
    system_time: u32,
    threads: [Option<TCB>; MAX_THREADS],
    atomics: [G8torAtomic; NUM_ATOMICS],
    atomic_mask: u8,
    stacks: MaybeUninit<[[u32; STACK_SIZE]; MAX_THREADS]>,
    peripherals: cortex_m::Peripherals,
}

#[exception]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    let _hfsr = (*cortex_m::peripheral::SCB::PTR).hfsr.read();
    let _cfsr = (*cortex_m::peripheral::SCB::PTR).cfsr.read();

    cortex_m::asm::nop();
    loop {}
}

const _: () = {
    let _ = cortex_m_rt::Exception::SysTick;
    let _ = cortex_m_rt::Exception::PendSV;
};

#[no_mangle]
#[unsafe(naked)]
pub unsafe extern "C" fn SysTick() {
    naked_asm!(
        "ldr r0, ={G8TOR_RTOS}",    // G8torRtos* r0 = &G8TOR_RTOS
        "cpsid i",                  // Begin critical section
        "ldr r1, [r0, #4]",         // u32 r1 = G8TOR_RTOS.system_time
        "add r1, r1, #1",           // r1++
        "str r1, [r0, #4]",         // G8TOR_RTOS.system_time = r1
        "cpsie i",                  // End critical section

        // Set the PENDSVSET bit to delay the context switch until after all
        // other interrupts have been serviced.
        "ldr r0, =0xE000ED04",
        "ldr r1, =0x10000000",
        "str r1, [r0]",
        "bx lr",
        G8TOR_RTOS = sym G8TOR_RTOS
    )
}

#[no_mangle]
#[unsafe(naked)]
pub unsafe extern "C" fn PendSV() {
    naked_asm!(
        "cpsid i",              // Begin critical section
        "push {{r4-r11}}",      // Save callee-saved registers (for the current thread)
        "ldr r0, ={G8TOR_RTOS}",  // G8torRtos* r0 = &G8TOR_RTOS
        "ldr r1, [r0, #0]",     // TCB* r1 = G8TOR_RTOS.running
        "str sp, [r1, #4]",     // u32* r1->sp = sp
        "push {{r0, lr}}",      // Save G8torRtos* r0 on stack (and lr for alignment)
        "bl {SCHEDULER}",       // Call scheduler (r0 = *G8torRtos)
        "pop {{r0, lr}}",       // Restore G8torRtos* r0 from stack (and lr for alignment)
        "ldr r1, [r0, #0]",     // TCB* r1 = G8TOR_RTOS.running
        "ldr sp, [r1, #4]",     // u32* sp = r1->sp
        "pop {{r4-r11}}",       // Restore callee-saved registers (for the new thread)
        "cpsie i",              // Enable interrupts (guarantees the next instruction is not interrupted)
        "bx lr",                // Branch to the new thread's PC (in lr) and restore caller-saved registers
        G8TOR_RTOS = sym G8TOR_RTOS,
        SCHEDULER = sym G8torRtos::scheduler
    );
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
        (&raw mut (*ptr).running).write(core::ptr::NonNull::dangling());
        (&raw mut (*ptr).system_time).write(0);
        (&raw mut (*ptr).threads).write([const { None }; MAX_THREADS]);
        (&raw mut (*ptr).atomics).write(
            [const {
                G8torAtomic {
                    count: ManuallyDrop::new(AtomicU8::new(0)),
                }
            }; NUM_ATOMICS],
        );
        (&raw mut (*ptr).stacks).write(MaybeUninit::uninit());
        (&raw mut (*ptr).peripherals).write(peripherals);
        // G8TOR_RTOS is now fully initialized
        // Except running which will be set when launching

        // Return a mutable reference to the static instance
        // To enable user code to add threads
        G8TOR_RTOS.assume_init_mut()
    }

    extern "C" fn scheduler(&mut self) {
        // Context switch logic
        self.running = unsafe { self.running.read().next };
    }

    /// Add a thread to the RTOS
    /// Safety: This function should only be called, at the start of the program BEFORE launch
    pub fn add_thread(&mut self, name: &str, thread: extern "C" fn() -> !) -> Result<(), ()> {
        if name.len() > 8 {
            return Err(()); // Name too long
        }

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
                    sp.add(8).write(0x00000000); // R0
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
                    id: id as u32,
                    sp: NonNull::new(sp as *mut u32)
                        .expect("Obviously sp is not null because we just wrote to it."),
                    next: NonNull::dangling(),
                    prev: NonNull::dangling(),
                    priority: 0,
                    blocked_by: None,
                    sleep_until: 0,
                    name: name.as_bytes().try_into().unwrap(),
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
                self.atomics[index].count = ManuallyDrop::new(AtomicU8::new(initial_count));
                return Ok(G8torSemaphoreHandle {
                    index: NonZeroU8::new((index + 1) as u8).unwrap(),
                    max_count,
                });
            }
        }

        Err(())
    }

    pub fn signal_semaphore(&self, handle: G8torSemaphoreHandle) {
        let index = (handle.index.get() - 1) as usize;
        let atomic = &self.atomics[index];

        // Increment the count atomically
        let count = unsafe { &atomic.count }; // SAFETY: Because the SemaphoreHandle was obtained from init_semaphore, we know that this atomic is a count

        let res = count.fetch_update(
            core::sync::atomic::Ordering::SeqCst,
            core::sync::atomic::Ordering::SeqCst,
            |current| {
                if current < handle.max_count {
                    Some(current + 1)
                } else {
                    None
                }
            },
        );

        match res {
            Ok(0) => {
                // There was possibly a thread blocked on this semaphore
                // Wake up one blocked thread
                for tcb_opt in self.threads.iter() {
                    if let Some(tcb) = tcb_opt {
                        if let Some(blocking_atomic) = tcb.blocked_by {
                            if blocking_atomic.index == handle.index {
                                // Unblock this thread
                                unsafe {
                                    let tcb_mut =
                                        &mut *(tcb as *const TCB as *mut TCB); // SAFETY: We have exclusive access to the RTOS and thus to all TCBs
                                    tcb_mut.blocked_by = None;
                                }
                                break; // Only unblock one thread
                            }
                        }
                    }
                }
            },
            Ok(_) => {},
            Err(_) => {
                // Semaphore was already at max count so block the current thread and yield
                let current_tcb = unsafe { self.running.as_mut() }; // SAFETY: We
            },
        }
    }

    pub fn init_mutex(&mut self) -> Result<G8torMutexHandle, ()> {
        for index in 0..NUM_ATOMICS {
            if (self.atomic_mask & (1 << index)) == 0 {
                // Found an empty atomic slot
                self.atomic_mask |= 1 << index;
                self.atomics[index].mutex = ManuallyDrop::new(AtomicBool::new(true));
                return Ok(G8torMutexHandle {
                    index: NonZeroU8::new((index + 1) as u8).unwrap(),
                });
            }
        }

        Err(())
    }

    pub unsafe fn launch(&mut self) -> ! {
        let _self_ptr = self as *mut Self;

        // Set the currently running thread to the first thread added
        self.running = match (*_self_ptr).threads[0].as_mut() {
            Some(tcb) => NonNull::new_unchecked(tcb as *mut TCB),
            None => panic!("No threads to run!"),
        };

        // Start SysTick
        let syst = &mut self.peripherals.SYST;
        let scb = &mut self.peripherals.SCB;
        syst.disable_counter();

        syst.clear_current();
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(cortex_m::peripheral::SYST::get_ticks_per_10ms() / 10 - 1); // 1 ms
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
            sp = in(reg) self.running.as_ptr(),
            options(noreturn)
        )
    }
}
