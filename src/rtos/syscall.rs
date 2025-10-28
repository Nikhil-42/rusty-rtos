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
                if let Some(blocker) = unsafe { (*t).blocked_by } {
                    if blocker.indexp1.get() - 1 == sem_index {
                        // Unblock and break
                        unsafe {
                            (*t).blocked_by = None;
                        }
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

// Does not execute in critical section so we must create a critical section to access G8TOR_RTOS
#[allow(unused_variables)]
pub(super) unsafe extern "C" fn _syscall(r0: usize, r1: usize, r2: usize, r3: usize, imm: u8) -> usize {
    let rtos = &raw mut G8TOR_RTOS as *mut G8torRtos; // SAFETY: the RTOS is definitely running so this is valid
    let running_tcb = (*rtos).running.unwrap_unchecked().as_ptr(); // SAFETY: the running thread called this syscall
    match imm {
        0 => sleep(rtos, running_tcb, r0 as u32),
        1 => wait_semaphore(rtos, running_tcb, r0 as u8),
        2 => signal_semaphore(rtos, running_tcb, r0 as u8),
        _ => unreachable!(), // Unknown syscall
    }
}
