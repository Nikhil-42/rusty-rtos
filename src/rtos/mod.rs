mod atomics;
mod handlers;

use crate::rtos::atomics::G8torAtomicHandle;
pub use crate::rtos::atomics::{
    G8torMutex, G8torMutexHandle, G8torMutexLock, G8torSemaphoreHandle,
};
use crate::rtos::handlers::init_idle;
use crate::syscall;

use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::sync::atomic::AtomicU8;
use core::{arch::asm, ptr::NonNull};

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;

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

impl<'a> IntoIterator for &'a mut TCB {
    type Item = &'a mut TCB;
    type IntoIter = TCBIterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        TCBIterator {
            end: self.prev,
            current: Some(NonNull::from(&*self)),
            _marker: PhantomData,
        }
    }
}

struct TCBIterator<'a> {
    end: NonNull<TCB>,
    current: Option<NonNull<TCB>>,
    _marker: PhantomData<&'a mut TCB>,
}

impl<'a> Iterator for TCBIterator<'a> {
    type Item = &'a mut TCB;

    fn next(&mut self) -> Option<&'a mut TCB> {
        let mut curr = self.current?;
        let next= unsafe { curr.as_ref() }.next;
        if curr == self.end {
            self.current = None;
        } else {
            self.current = Some(next);
        }
        // Move to the next TCB
        Some(unsafe { curr.as_mut() } )
    }
}

#[repr(C)]
pub struct G8torRtos {
    running: Option<NonNull<TCB>>,
    system_time: u32,
    threads: [Option<TCB>; MAX_THREADS],
    atomics: [AtomicU8; NUM_ATOMICS],
    atomic_mask: u8,
    stacks: MaybeUninit<[[u32; STACK_SIZE]; MAX_THREADS]>,
    peripherals: cortex_m::Peripherals,
}

// Executes in critical section so we have exclusive access to G8TOR_RTOS
unsafe extern "C" fn _scheduler(rtos: &mut G8torRtos) -> Option<&mut TCB> {
    let next_thread = if let Some(running) = rtos.running.as_mut() {
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

    let mut selected: Option<&mut TCB> = None;
    for thread in next_thread {
        if thread.asleep {
            // Check if the deadline has passed
            let current_time = rtos.system_time;
            if thread.sleep_until.wrapping_sub(current_time) as i32 <= 0 {
                // Wake up the thread
                thread.asleep = false;
            } else {
                continue;
            }
        }
        // next_thread is not asleep

        if thread.blocked_by.is_some() {
            continue;
        }
        // next_thread is not blocked

        if let Some(sel) = selected {
            // We have a candidate thread, compare priorities
            if thread.priority < sel.priority {
                selected = Some(thread);
            } else {
                selected = Some(sel);
            }
        } else {
            // No candidate thread yet, select this one
            selected = Some(thread);
        }
    }

    return selected;
}

#[allow(unused_variables)]
unsafe extern "C" fn _syscall(r0: usize, r1: usize, r2: usize, r3: usize, imm: u8) -> usize {
    let rtos = &raw mut G8TOR_RTOS as *mut G8torRtos; // SAFETY: the RTOS is definitely running
    let running_tcb = (*rtos).running.unwrap_unchecked().as_ptr(); // SAFETY: something called this function
    match imm {
        0 => {
            // Sleep
            let sleep_time = r0 as u32;

            // Pend a context switch
            cortex_m::peripheral::SCB::set_pendsv();

            if r0 > 0 {
                // Sleep syscall
                let system_time = &raw const (*rtos).system_time;
                let current_time = core::ptr::read_volatile(system_time);
                let wake_time = current_time.wrapping_add(sleep_time);

                // SAFTEY: This is safe because PendSV will not run until after we exit the syscall
                // so the running thread cannot change while we are in this function
                (*running_tcb).asleep = true;
                (*running_tcb).sleep_until = wake_time;
            }

            0
        }
        1 => {
            // Wait Semaphore
            let rtos = rtos as *mut G8torRtos;
            let sem_index = r0 as u8;
            let max_count = r1 as u8;

            cortex_m::interrupt::free(|_cs| {
                let prev_count = (*rtos).atomics[sem_index as usize].fetch_update(
                    core::sync::atomic::Ordering::SeqCst,
                    core::sync::atomic::Ordering::SeqCst,
                    |count| {
                        if count > 0 {
                            Some(count - 1)
                        } else {
                            None
                        }
                    },
                );

                (match prev_count {
                    Ok(val) if val == max_count => {
                        // Unblock blocked thread
                        let mut t = (*running_tcb).next.as_ptr();
                        while t != running_tcb {
                            if let Some(blocker) = (*t).blocked_by {
                                if blocker.indexp1.get() - 1 == sem_index {
                                    // Unblock and break
                                    (*t).blocked_by = None;
                                    break;
                                }
                            }
                            t = (*t).next.as_ptr();
                        }
                        val
                    }
                    Err(val) => {
                        // Block current thread
                        // Pend a context switch
                        cortex_m::peripheral::SCB::set_pendsv();
                        (*running_tcb).blocked_by = Some(G8torAtomicHandle::from_index(sem_index));
                        val
                    }
                    Ok(val) => val,
                }) as usize
            })
        }
        2 => {
            // Signal Semphore
            let rtos = rtos as *mut G8torRtos;
            let sem_index = r0 as u8;
            let max_count = r1 as u8;

            cortex_m::interrupt::free(|_cs| {
                let prev_count = (*rtos).atomics[sem_index as usize].fetch_update(
                    core::sync::atomic::Ordering::SeqCst,
                    core::sync::atomic::Ordering::SeqCst,
                    |count| {
                        if count < max_count {
                            Some(count + 1)
                        } else {
                            None
                        }
                    },
                );

                (match prev_count {
                    Ok(val) if val == 0 => {
                        // Unblock blocked thread
                        let mut t = (*running_tcb).next.as_ptr();
                        while t != running_tcb {
                            if let Some(blocker) = (*t).blocked_by {
                                if blocker.indexp1.get() - 1 == sem_index {
                                    // Unblock and break
                                    (*t).blocked_by = None;
                                    break;
                                }
                            }
                            t = (*t).next.as_ptr();
                        }
                        val
                    }
                    Err(val) => {
                        // Block current thread
                        // Pend a context switch
                        cortex_m::peripheral::SCB::set_pendsv();
                        (*running_tcb).blocked_by = Some(G8torAtomicHandle::from_index(sem_index));
                        val
                    }
                    Ok(val) => val,
                }) as usize
            })
        }

        _ => unreachable!(), // Unknown syscall
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
        (&raw mut (*ptr).atomics).write([const { AtomicU8::new(0) }; NUM_ATOMICS]);
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
    pub fn add_thread(
        &mut self,
        name: &[u8; NAME_LEN],
        priority: u8,
        thread: extern "C" fn(G8torRtosHandle) -> !,
    ) -> Result<(), ()> {
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
                    priority: priority,
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

                    // Release the thread reference
                    let thread_ptr = thread as *mut TCB;

                    // Update the previous TCB's next pointer to point to the new TCB
                    unsafe {
                        self.threads[id - 1].as_mut().unwrap_unchecked().next =
                            NonNull::new_unchecked(thread_ptr)
                    }; // SAFETY: We just checked that i > 0 also id = i implies that threads[k] is Some for k < i

                    // Update the first TCB's prev pointer to point to the new TCB
                    unsafe {
                        self.threads[0].as_mut().unwrap_unchecked().prev =
                            NonNull::new_unchecked(thread_ptr)
                    }; // SAFETY: NUM_THREADS > 0 and id = i implies that threads[k] is Some for k < i
                       // so k = 0 is definitely Some
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
                self.atomics[index] = AtomicU8::new(initial_count.min(max_count));
                return Ok(G8torSemaphoreHandle {
                    index: index as u8,
                    max_count,
                });
            }
        }

        // No empty atomic slot found
        Err(())
    }

    pub fn init_mutex<T>(
        &mut self,
        mutex: &'static G8torMutex<T>,
    ) -> Result<G8torMutexHandle<T>, ()> {
        for index in 0..NUM_ATOMICS {
            if (self.atomic_mask & (1 << index)) == 0 {
                // Found an empty atomic slot
                self.atomic_mask |= 1 << index;
                self.atomics[index] = AtomicU8::new(1); // Mutex is initially unlocked
                return Ok(G8torMutexHandle {
                    index: index as u8,
                    mutex,
                });
            }
        }

        // No empty atomic slot found
        Err(())
    }

    pub unsafe fn launch(&mut self) -> ! {
        let _self_ptr = self as *mut Self;

        // Configure the idle thread jump address
        init_idle();

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
        syst.set_clock_source(SystClkSource::Core); // External maps to the PIOSC / 4 (4 MHz)
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
            "ldr sp, =0x20008000",  // Set MSP to top of RAM (nuke any previous stack)
            "ldr r0, [{tcb}, #4]",  // u32* r0 = self.running->sp
            "msr psp, r0",          // Set PSP to the thread's stack pointer
            "mrs r0, CONTROL",      // read->modify->write CONTROL
            "orr r0, r0, #0x2",     // Switch to PSP
            "msr CONTROL, r0",      // write CONTROL
            "isb",                  // Instruction sync barrier
            // sp = PSP now
            "pop {{r4-r11}}",       // Restore callee-saved registers
            "pop {{r0-r3, r12}}",   // Restore caller-saved registers
            "add sp, sp, #4",       // Skip LR
            "pop {{lr}}",           // Restore pc into lr
            "add sp, sp, #4",       // Skip xPSR
            "cpsie i",              // Enable interrupts
            "bx lr",                // Branch to the thread's PC
            tcb = in(reg) self.running.unwrap_unchecked().as_ptr(),
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

    pub fn wait_semaphore(&self, sem: G8torSemaphoreHandle) -> u8 {
        syscall!(1; sem.index as usize, sem.max_count as usize) as u8
    }

    pub fn signal_semaphore(&self, sem: G8torSemaphoreHandle) -> u8 {
        syscall!(2; sem.index as usize, sem.max_count as usize) as u8
    }

    pub fn take_mutex<T>(&self, handle: &G8torMutexHandle<T>) -> G8torMutexLock<T> {
        // Functionally the same as wait_semaphore
        syscall!(1; handle.index as usize, u8::MAX as usize); // Never block on release mutex

        // Successfully took the mutex
        return G8torMutexLock {
            mutex: handle.mutex,
        };
    }

    pub fn release_mutex<T>(&self, handle: &G8torMutexHandle<T>, lock: G8torMutexLock<T>) {
        if core::ptr::addr_eq(handle.mutex, lock.mutex) {
            syscall!(2; handle.index as usize, u8::MAX as usize); // Never block on release mutex
        }
        else {
            panic!("Attempted to release a mutex with a lock from a different mutex!");
        }
    }
}
