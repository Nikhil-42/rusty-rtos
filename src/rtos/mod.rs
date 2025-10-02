use core::arch::naked_asm;
use core::cell::UnsafeCell;
use core::{arch::asm, ptr::NonNull};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::scb::{Exception, SystemHandler};
use cortex_m_rt::{exception, ExceptionFrame};

const MAX_THREADS: usize = 2;
const STACK_SIZE: usize = 32;

static mut G8TOR_RTOS: Option<G8torRtos> = None;

#[repr(C)]
pub struct TCB {
    pub id: u32,
    pub sp: NonNull<u32>, // Stack pointer (points to top of stack)
    pub next: NonNull<TCB>,
    pub prev: NonNull<TCB>,
}

#[repr(C)]
pub struct G8torRtos {
    running: NonNull<TCB>,
    system_time: u32,
    threads: [Option<TCB>; MAX_THREADS],
    stacks: [[u32; STACK_SIZE]; MAX_THREADS],
    peripherals: cortex_m::Peripherals,
}

// #[unsafe(naked)]
// unsafe extern "C" fn SysTick() {
//     naked_asm!(
//         "nop"
//     );
// }

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    cortex_m::asm::nop();
    let hfsr = (*cortex_m::peripheral::SCB::PTR).hfsr.read();
    let cfsr = (*cortex_m::peripheral::SCB::PTR).cfsr.read();

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
    pub unsafe fn new(peripherals: cortex_m::Peripherals) -> &'static mut Self {
        // Ensure only one instance is created
        if let Some(_) = G8TOR_RTOS {
            panic!("RTOS already initialized!");
        }

        // Disable interrupts during initialization
        cortex_m::interrupt::disable();

        // SAFETY: We only write to G8TOR_RTOS here, and no other references exist.
        let rtos = Some(Self {
            threads: [const { None }; MAX_THREADS],
            stacks: [[0xCC; STACK_SIZE]; MAX_THREADS],
            system_time: 0,
            running: core::ptr::NonNull::dangling(),
            peripherals,
        });

        cortex_m::asm::nop();
        (&raw mut G8TOR_RTOS).write(rtos);

        G8TOR_RTOS.as_mut().unwrap_unchecked()
    }

    extern "C" fn scheduler(&mut self) {
        // Context switch logic
        self.running = unsafe { self.running.read().next };
    }

    /// Add a thread to the RTOS
    /// Safety: This function should only be called, at the start of the program BEFORE launch
    pub unsafe fn add_thread(&mut self, thread: extern "C" fn() -> !) -> Result<(), ()> {
        // Find an empty TCB slot
        let _self_ptr = self as *mut Self;
        for id in 0..MAX_THREADS {
            if self.threads[id].is_none() {
                // Initialize the stack for the new thread
                let stack = &mut self.stacks[id] as *mut [u32; STACK_SIZE];
                let sp = stack.add(STACK_SIZE);
                let sp = sp.sub(16); // Reserve const DEFAULT_TCB: TCB = TCB {

                // Set up initial stack frame
                (*sp)[15] = 0x01000000; // xPSR
                (*sp)[14] = thread as u32; // PC
                (*sp)[13] = 0x14141414; // R14 (LR)
                (*sp)[12] = 0x12121212; // R12
                (*sp)[11] = 0x03030303; // R3
                (*sp)[10] = 0x02020202; // R2
                (*sp)[9]  = 0x01010101; // R1
                (*sp)[8]  = 0x00000000; // R0
                (*sp)[7]  = 0x11111111;  // R11
                (*sp)[6]  = 0x10101010; // R10
                (*sp)[5]  = 0x09090909; // R9
                (*sp)[4]  = 0x08080808; // R8
                (*sp)[3]  = 0x07070707; // R7
                (*sp)[2]  = 0x06060606; // R6
                (*sp)[1]  = 0x05050505; // R5
                (*sp)[0]  = 0x04040404; // R4

                // Create the new TCB and link it into the circular doubly linked list
                let thread = self.threads[id].insert(TCB {
                    sp: NonNull::new_unchecked(sp as *mut u32),
                    id: id as u32,
                    next: NonNull::dangling(),
                    prev: NonNull::dangling(),
                });

                if id != 0 {
                    // Not the first thread, link it to the previous thread and the first thread
                    thread.next =
                        NonNull::new_unchecked((*_self_ptr).threads[0].as_mut().unwrap_unchecked());
                    // SAFETY: We just checked that i > 0 also id = i implies that threads[k] is Some for k < i
                    // so k = 0 is definitely Some and k = i-1 is definitely Some
                    thread.prev =
                        NonNull::new_unchecked((*_self_ptr).threads[id - 1].as_mut().unwrap_unchecked());
                    // Update the previous TCB's next pointer to point to the new TCB
                    self.threads[id - 1].as_mut().unwrap_unchecked().next =
                        NonNull::new_unchecked(thread as *mut TCB);
                } else {
                    // First thread, points to itself
                    // SAFETY: We just checked that i > 0 also id = i implies that threads[k] is Some for k < i
                    // so k = 0 is definitely Some
                    thread.next = NonNull::new_unchecked(thread as *mut TCB);
                    thread.prev = NonNull::new_unchecked(thread as *mut TCB);
                }

                return Ok(());
            }
        }
        return Err(()); // No empty slot found
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
        // syst.set_reload(2000);
        unsafe {
            scb.set_priority(SystemHandler::SysTick, 0); // Highest priority
            scb.set_priority(SystemHandler::PendSV, 0b11100000); // Lowest priority
        };
        syst.enable_interrupt();  // Note: interrupts are still disabled globally
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
            sp = in(reg) self.running.as_ptr()
        );

        cortex_m::asm::udf(); // Should never reach here
    }
}
