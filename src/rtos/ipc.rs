use core::mem::MaybeUninit;

use crate::rtos::{G8torMutex, G8torMutexLock};

pub struct G8torFifoHandle {
    pub(super) index: u8,
}

pub(super) struct G8torFifoInternals<const LEN: usize> {
    head: usize,
    tail: usize,
    buffer: [u32; LEN],
}

pub(super) struct G8torFifo<const LEN: usize> {
    pub(super) internals: G8torMutex<G8torFifoInternals<LEN>>,
    pub(super) semaphore_idx: u8,
    pub(super) mutex_idx: u8,
}

impl<const LEN: usize> G8torFifo<LEN> {
    pub const fn new(mutex_idx: u8, semaphore_idx: u8) -> Self {
        G8torFifo {
            internals: G8torMutex::new(G8torFifoInternals {
                head: 0,
                tail: 0,
                buffer: [0; LEN],
            }),
            semaphore_idx,
            mutex_idx,
        }
    }

    pub fn read(&'static self, lock: G8torMutexLock<G8torFifoInternals<LEN>>) -> (G8torMutexLock<G8torFifoInternals<LEN>>, u32) {
        let internals = self.internals.get(lock);
        let val = internals.buffer[internals.head];
        internals.head = (internals.head + 1) % LEN;
        
        let lock = self.internals.release(internals);
        (lock, val)
    }

    pub fn write(&'static self, lock: G8torMutexLock<G8torFifoInternals<LEN>>, val: u32) -> G8torMutexLock<G8torFifoInternals<LEN>> {
        let internals = self.internals.get(lock);
        internals.buffer[internals.tail] = val;
        internals.tail = (internals.tail + 1) % LEN;

        self.internals.release(internals)
    }
}
