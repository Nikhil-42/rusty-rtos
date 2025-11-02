use core::ptr::NonNull;

use super::{G8torRtos, TCB, NAME_LEN};

// Executes in critical section so we have exclusive access to G8TOR_RTOS
pub(super) unsafe extern "C" fn _scheduler(rtos: *mut G8torRtos) -> Option<NonNull<TCB<NAME_LEN>>> {
    let rtos = &mut *rtos;
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

    let mut selected: Option<&mut TCB<NAME_LEN>> = None;
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

    return core::mem::transmute(selected);
}