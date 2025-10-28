use core::ptr::NonNull;

use crate::rtos::MAX_THREADS;

use super::{G8TOR_RTOS, NAME_LEN};
use super::{G8torRtos, G8torAtomicHandle, TCB};

#[inline(always)]
fn sleep(rtos: *mut G8torRtos, running_tcb: *mut TCB<NAME_LEN>, duration: u32) -> usize {
    // Pend a context switch
    cortex_m::peripheral::SCB::set_pendsv();

    if duration > 0 {
        // Sleep syscall
        let system_time = unsafe { &raw const (*rtos).system_time };        // SAFETY: just calculating address doesn't access data
        let current_time = unsafe { core::ptr::read_volatile(system_time) };  // SAFETY: system_time is 32-bit aligned u32 so it is safe to read atomically
        let wake_time = current_time.wrapping_add(duration);

        // SAFTEY: This is safe because PendSV will not run until after we exit the syscall
        // so the running thread cannot change while we are in this function
        unsafe {
            (*running_tcb).asleep = true;
            (*running_tcb).sleep_until = wake_time;
        }
    }

    0
}

#[inline(always)]
fn wait_semaphore(rtos: *mut G8torRtos, running_tcb: *mut TCB<NAME_LEN>, sem_index: u8) -> usize {
    cortex_m::interrupt::free(|_cs| {
        let prev_count = unsafe { (*rtos).atomics[sem_index as usize] };

        let new_count = if prev_count == 0 {
            // Block current thread
            // Pend a context switch
            cortex_m::peripheral::SCB::set_pendsv();
            unsafe {
                (*running_tcb).blocked_by = Some(G8torAtomicHandle::from_index(sem_index));
            }

            0
        } else {
            prev_count - 1
        };
        unsafe {
            (*rtos).atomics[sem_index as usize] = new_count;
        }

        prev_count as usize
    })
}

#[inline(always)]
fn signal_semaphore(rtos: *mut G8torRtos, running_tcb: *mut TCB<NAME_LEN>, sem_index: u8) -> usize {
    cortex_m::interrupt::free(|_cs| {
        let prev_count = unsafe { (*rtos).atomics[sem_index as usize] };

        // If we were at 0, we are adding to an empty semaphore (move the resource )
        let unblocked = if prev_count == 0 {
            // Unblock blocked thread
            let next_thread = unsafe { (*running_tcb).next.as_mut() };

            let mut unblocked= false;
            for t in next_thread {
                if let Some(blocker) = t.blocked_by {
                    if blocker.indexp1.get() - 1 == sem_index {
                        // Unblock and break
                        t.blocked_by = None;
                        unblocked = true;
                        break;
                    }
                }
            }

            unblocked
        } else {
            false
        };

        unsafe {
            (*rtos).atomics[sem_index as usize] = if unblocked { prev_count } else { prev_count.saturating_add(1) };
        }

        prev_count as usize
    })
}

#[inline(always)]
fn spawn_thread(rtos: *mut G8torRtos, name: &[u8; NAME_LEN], priority: u8, thread: extern "C" fn(super::G8torRtosHandle) -> !) -> usize {
    cortex_m::interrupt::free(|_cs| {
        // SAFETY: This is safe because we are in a critical section
        let rtos = unsafe { &mut *rtos };

        let id = match (0..super::MAX_THREADS).find(|&id| rtos.threads[id].is_none()) {
            Some(id) => id,
            None => return 1, // No empty TCB slots
        };

        // Initialize the stack for the new thread
        // This is okay because self.stacks is a MaybeUninit and we only write to it here
        let stack = rtos.stacks.as_mut_ptr().cast::<[u32; super::STACK_SIZE]>();
        let stack = unsafe { stack.add(id) }; // SAFETY: id < MAX_THREADS so this is in bounds
        let sp = unsafe { stack.cast::<u32>().add(super::STACK_SIZE) }; // SAFETY: stack is valid and STACK_SIZE is in bounds

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

        let link_target_id = match id {
            0 => {
                (1..MAX_THREADS).rev().find(|tid| rtos.threads[*tid].is_some())
            },
            id => {
                Some(id - 1)
            }
        };

        // Create the new TCB and link it into the circular doubly linked list
        let thread = rtos.threads[id].insert(TCB {
            id,
            sp: NonNull::new(sp as *mut u32)
                .expect("Obviously sp is not null because we just wrote to it."),
            next: NonNull::dangling(), // Will be set below
            prev: NonNull::dangling(), // Will be set below
            sleep_until: 0,
            asleep: false,
            priority,
            blocked_by: None,
            name: *name,
        }) as *mut TCB<NAME_LEN>;

        if let Some(tid) = link_target_id {
            // Link after the target thread
            let target_tcb = unsafe {
                rtos.threads[tid].as_mut().unwrap_unchecked()
            }; // SAFETY: We just found that tid is Some

            // Set the new thread's next and prev pointers
            unsafe { 
                let next_ptr = target_tcb.next.as_ptr();

                (*thread).next = NonNull::new_unchecked(next_ptr);
                (*thread).prev = NonNull::from_mut(target_tcb);

                (*next_ptr).prev = NonNull::new_unchecked(thread);
                target_tcb.next = NonNull::new_unchecked(thread);
            };
        } else {
            // First thread, points to itself
            unsafe {
                (*thread).next = NonNull::new_unchecked(thread);
                (*thread).prev = NonNull::new_unchecked(thread);
            };
        }

        return 0;
    })
}

#[inline(always)]
fn kill_thread(_rtos: *mut G8torRtos, running_tcb: *mut TCB<NAME_LEN>, thread_id: usize) -> usize {
    // SAFETY: This is safe because PendSV will not run until after we exit the syscall
    let mut to_remove= None;
    for t in unsafe { &mut *running_tcb } {
        if t.id == thread_id {
            to_remove = Some(t as *mut TCB<NAME_LEN>);
            break;
        }
    }

    if let Some(t) = to_remove {
        unsafe { 
            (*t).prev.as_mut().next = (*t).next;
            (*t).next.as_mut().prev = (*t).prev;
        }

        if unsafe { (*t).id == (*running_tcb).id } {
            // We are killing the running thread, pend a context switch
            cortex_m::peripheral::SCB::set_pendsv();
        }

        // SAFETY: We are the only ones with access to the TCB right now
        //           because we are in the SV handler > PendSV
        // SAFETY: Option is transparent over TCB so this is valid
        unsafe { *(t as *mut TCB<NAME_LEN> as *mut Option<TCB<NAME_LEN>>) = None };
        
        0
    } else {
        1 // Thread not found
    }
}
// Does not execute in critical section so we must create a critical section to access G8TOR_RTOS
#[allow(unused_variables)]
pub(super) unsafe extern "C" fn _syscall(r0: usize, r1: usize, r2: usize, r3: usize, imm: u8) -> usize {
    let rtos = &raw mut G8TOR_RTOS as *mut G8torRtos; // SAFETY: the RTOS is definitely running so this is valid
    let running_tcb = (*rtos).running.unwrap_unchecked().as_ptr(); // SAFETY: the running thread called this syscall
    match imm {
        0 => sleep(rtos, running_tcb, r0 as u32),
        1 => wait_semaphore(rtos, running_tcb, r0 as u8),
        2 => signal_semaphore(rtos, running_tcb, r0 as u8),
        254 => spawn_thread(rtos, &*(r0 as *const [u8; NAME_LEN]), r1 as u8, core::mem::transmute(r2)),
        255 => kill_thread(rtos, running_tcb, r0 as usize),
        _ => unreachable!(), // Unknown syscall
    }
}
