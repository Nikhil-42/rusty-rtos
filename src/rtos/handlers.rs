use core::{arch::{asm, naked_asm}, mem::offset_of};
use cortex_m_rt::{exception, ExceptionFrame};

use super::{periodic::_run_periodics, scheduler::_scheduler, syscall::_syscall, G8TOR_RTOS};

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    let hfsr = (*cortex_m::peripheral::SCB::PTR).hfsr.read();
    let cfsr = (*cortex_m::peripheral::SCB::PTR).cfsr.read();

    #[allow(unused_variables)]
    let info = core::hint::black_box((hfsr, cfsr, ef));
    asm!("bkpt #0");
    loop {}
}

const _: () = {
    let _ = cortex_m_rt::Exception::SysTick;
    let _ = cortex_m_rt::Exception::PendSV;
};

#[no_mangle]
#[unsafe(naked)]
unsafe extern "C" fn SysTick() {
    naked_asm!(
        "ldr r0, ={G8TOR_RTOS}",    // G8torRtos* r0 = &G8TOR_RTOS

        // Call periodic task runner
        "push {{r0, lr}}",              // Save lr
        "bl {PERIODIC_RUNNER}",         // _run_periodics(r0 = &G8TOR_RTOS)
        "pop {{r0, lr}}",               // Restore lr

        // This is safe because no other code modifies system_time
        // This also makes it always illegal to take a reference to G8TOR_RTOS
        "ldr r1, [r0, {G8torRtos_system_time}]",         // u32 r1 = G8TOR_RTOS.system_time
        "add r1, r1, #1",           // r1++
        "str r1, [r0, {G8torRtos_system_time}]",         // G8TOR_RTOS.system_time = r1

        // Set the PENDSVSET bit to delay the context switch until after all
        // other interrupts have been serviced.
        "ldr r0, =0xE000ED04",
        "ldr r1, =0x10000000",
        "str r1, [r0]",

        "bx lr",
        G8TOR_RTOS = sym G8TOR_RTOS,
        G8torRtos_system_time = const offset_of!(super::G8torRtos, system_time),
        PERIODIC_RUNNER = sym _run_periodics
    )
}

#[no_mangle]
#[unsafe(naked)]
unsafe extern "C" fn PendSV() {
    naked_asm!(
        "cpsid i",              // Begin critical section
        // check if there is a running thread
        "ldr r0, ={G8TOR_RTOS}",  // G8torRtos* r0 = &G8TOR_RTOS
        "ldr r1, [r0, {G8torRtos_running}]",     // TCB* r1 = G8TOR_RTOS.running
        "cbz r1, 1f",           // if (r1 == null) don't save context
        // G8TOR_RTOS.running is not null, save its context
        "mrs r12, psp",         // Get process stack pointer
        "stmdb r12!, {{r4-r11}}",      // Save callee-saved registers (for the current thread)
        "str r12, [r1, {TCB_sp}]",     // u32* r1->sp = sp
        "strb lr, [r1, {TCB_lr}]",     // u8 r1->lr = lr

        // Call scheduler to select the next thread to run
        "1:",
        "push {{r0, lr}}",      // Save G8torRtos* r0 on stack (and lr for alignment)
        "bl {SCHEDULER}",       // Call scheduler(r0 = *G8torRtos) => returns TCB* in r0
        "mov r1, r0",           // TCB* r1 = return value from scheduler
        "pop {{r0, lr}}",       // Restore G8torRtos* r0 from stack (and lr for alignment)
        "str r1, [r0, {G8torRtos_running}]",     // G8TOR_RTOS.running = r1

        // r1 may be null if no thread is ready to run
        "cbz r1, 1f",           // if (r1 == null) skip context restore

        // There is a thread to run, restore its context
        "ldr r12, [r1, {TCB_sp}]",    // u32* sp = r1->sp
        "ldrsb lr, [r1, {TCB_lr}]",    // u8 lr = r1->lr
        "ldm r12!, {{r4-r11}}",       // Restore callee-saved registers (for the new thread)
        "msr psp, r12",         // Set PSP to frame
        "cpsie i",              // End critical section
        "bx lr",                // Return from PendSV

        // No thread to run, enter low-power idle state
        "1:",
        "ldr r12, ={IDLE_FRAME}",     // Load address of idle frame
        "ldr lr, =0xFFFFFFFD", // Load LR for idle frame
        "msr psp, r12",         // Set PSP to frame
        "cpsie i",              // Enable interrupts
        "bx lr",                // Return from PendSV

        G8TOR_RTOS = sym G8TOR_RTOS,
        G8torRtos_running = const offset_of!(super::G8torRtos, running),
        SCHEDULER = sym _scheduler,
        IDLE_FRAME = sym IDLE_FRAME,
        TCB_sp = const offset_of!(super::TCB, sp),
        TCB_lr = const offset_of!(super::TCB, lr),
    );
}

#[no_mangle]
#[unsafe(naked)]
unsafe extern "C" fn idle() {
    naked_asm!(
        "1:", "wfi",  // Wait for interrupt
        "b 1b", // Repeat
    )
}

#[repr(C, align(8))]
struct IdleFrame {
    r0: u32, // id = -1
    r1: u32,
    r2: u32,
    r3: u32,
    r12: u32,
    lr: u32,   // Link register
    pc: u32,   // Program counter
    xpsr: u32, // Program status register
}

static mut IDLE_FRAME: IdleFrame = IdleFrame {
    r0: 0xFFFFFFFF,   // R0 (id = -1)
    r1: 0x01010101,   // R1
    r2: 0x02020202,   // R2
    r3: 0x03030303,   // R3
    r12: 0x12121212,  // R12
    lr: 0x14141414,   // R14 (LR)
    pc: 0x00000000,   // PC -- will be filled in with idle()
    xpsr: 0x01000000, // xPSR
};
pub fn init_idle() {
    unsafe {
        IDLE_FRAME.pc = idle as u32;
    }
}

#[macro_export]
macro_rules! syscall {
    ($imm:expr $(; $($arg:expr),* )? ) => {{
        #[allow(unused_macros)]
        macro_rules! call {
            () => { crate::rtos::handlers::_syscall0::<$imm>() };
            ($r0:expr) => { crate::rtos::handlers::_syscall1::<$imm>($r0) };
            ($r0:expr, $r1:expr) => { crate::rtos::handlers::_syscall2::<$imm>($r0, $r1)  };
            ($r0:expr, $r1:expr, $r2:expr) => { crate::rtos::handlers::_syscall3::<$imm>($r0, $r1, $r2)  };
            ($r0:expr, $r1:expr, $r2:expr, $r3:expr) => { crate::rtos::handlers::_syscall4::<$imm>($r0, $r1, $r2, $r3)  };
        }

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        let v = call!($($($arg),*)?);
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        v
    }};
}

macro_rules! gen_syscall {
    ($name:ident; $($rX:ident),*) => {
        #[unsafe(naked)]
        pub(super) extern "C" fn $name<const NUM: u8>($($rX: usize),*) -> usize {
            naked_asm!(
                "svc {num}",
                "nop",
                "bx lr",
                num = const NUM
            );
        }
    };
}

// Generate syscall functions for up to 4 arguments
gen_syscall!(_syscall4; r0, r1, r2, r3);
gen_syscall!(_syscall3; r0, r1, r2);
gen_syscall!(_syscall2; r0, r1);
gen_syscall!(_syscall1; r0);
gen_syscall!(_syscall0;);

/// SVCall trampoline to call syscall
#[no_mangle]
#[unsafe(naked)]
unsafe extern "C" fn SVCall() {
    naked_asm!(
        "tst lr, #4",           // Check EXC_RETURN bit 2 SPSEL
        "ite eq",
        "mrseq r12, msp",       // Load PSP into r12
        "mrsne r12, psp",       // Load MSP into r12
        // r12 now contains the interrupted stack pointer
        "ldr r1, [r12, #24]",   // Load the old PC into r1
        "ldrb r1, [r1, #-2]",   // Load the imm byte from instruction memory
        "push {{r12, lr}}",       
        // r1 now contains the immediate value
        // extern "C" syscall(r0: usize, r1, usize, r2: usize, r3: usize, imm: u8) -> (usize, usize)
        "sub sp, sp, #8",       // Args for the syscall
        "str r1, [sp, #0]",
        "ldr r3, [r12, #12]",
        "ldr r2, [r12, #8]",
        "ldr r1, [r12, #4]",
        "ldr r0, [r12, #0]",
        "bl {SYSCALL}",
        "add sp, sp, #8",       // Clean up syscall args
        // r0 contains return value
        "pop {{r12, lr}}",
        "str r0, [r12, #0]",    // Overwrite stacked frame with return
        "bx lr",                // Return from SVC
        SYSCALL = sym _syscall
    );
}
