use core::arch::naked_asm;

use cortex_m_rt::{exception, ExceptionFrame};

use crate::rtos::{G8TOR_RTOS, _scheduler, _syscall};


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
unsafe extern "C" fn SysTick() {
    naked_asm!(
        "ldr r0, ={G8TOR_RTOS}",    // G8torRtos* r0 = &G8TOR_RTOS
        // This is safe because no other code modifies system_time
        // This also makes it always illegal to take a reference to G8TOR_RTOS
        "ldr r1, [r0, #4]",         // u32 r1 = G8TOR_RTOS.system_time
        "add r1, r1, #1",           // r1++
        "str r1, [r0, #4]",         // G8TOR_RTOS.system_time = r1

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
unsafe extern "C" fn PendSV() {
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
        SCHEDULER = sym _scheduler
    );
}

#[macro_export]
macro_rules! syscall {
    ($imm:expr $(; $($arg:expr),* )? ) => {{
        #[allow(unused_macros)]
        macro_rules! call {
            () => { unsafe { _syscall0::<$imm>() } };
            ($r0:expr) => { unsafe { _syscall1::<$imm>($r0) } };
            ($r0:expr, $r1:expr) => { unsafe { _syscall2::<$imm>($r0, $r1) } };
            ($r0:expr, $r1:expr, $r2:expr) => { unsafe { _syscall3::<$imm>($r0, $r1, $r2) } };
            ($r0:expr, $r1:expr, $r2:expr, $r3:expr) => { unsafe { _syscall4::<$imm>($r0, $r1, $r2, $r3) } };
        }
        call!($($($arg),*)?)
    }};
}

#[repr(C)]
pub struct SyscallReturn(usize, usize);
macro_rules! gen_syscall {
    ($name:ident; $($rX:ident),*) => {
        #[unsafe(naked)]
        extern "C" fn $name<const num: u8>($($rX: usize),*) -> SyscallReturn {
            naked_asm!("svc {num}", num = const num);
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
        "push {{r4, lr}}",
        "tst lr, #4",           // Check if bit 2 is set (to determine which stack)
        "ite eq",               // Mini-branch 
        "mrseq r4, msp",        // If MSP, load MSP into r4
        "mrsne   r4, psp",      // If PSP, load PSP into r4
        // r0 now contains the interrupted stack pointer
        "ldr r1, [r4, #24]",    // Load the old PC into r1
        "ldrb r1, [r1, #-2]",   // Load the imm byte from instruction memory
        // r1 now contains the immediate value
        "sub sp, sp, #8",       // Args for the syscall
        "str r1, [sp, #0]",     
        "ldr r3, [r4, #12]",    
        "ldr r2, [r4, #8]",
        "ldr r1, [r4, #4]",
        "ldr r0, [r4, #0]",
        // extern "C" syscall(r0: usize, r1, usize, r2: usize, r3: usize, imm: u8) -> (usize, usize)
        "bl {SYSCALL}",
        // r0 and r1 contain return value
        "str r0, [r4, #0]",     // Overwrite stacked frame with return
        "str r1, [r4, #4]",
        "pop {{r4, lr}}",
        "bx lr",                // Return from SVC
        SYSCALL = sym _syscall
    );
}
