use core::{mem::ManuallyDrop, num::NonZeroU8, sync::atomic::{AtomicBool, AtomicU8}, u8};

use tm4c123x_hal::pac::gpio_porta::im;


#[derive(Clone, Copy, Eq, PartialEq)]
pub(super) struct G8torAtomicHandle {
    pub(super) index: NonZeroU8,
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

impl From<usize> for G8torMutexHandle {
    #[inline(always)]
    fn from(index: usize) -> Self {
        assert!(index < u8::MAX as usize - 1);
        G8torMutexHandle {
            index: NonZeroU8::new((index as u8) + 1).unwrap(),
        }
    }
}

impl Into<usize> for G8torMutexHandle {
    #[inline(always)]
    fn into(self) -> usize {
        (self.index.get() - 1) as usize
    }
}

pub struct G8torMutexLock {}

#[derive(Clone, Copy)]
pub struct G8torSemaphoreHandle {
    index: NonZeroU8,
}

impl From<usize> for G8torSemaphoreHandle {
    #[inline(always)]
    fn from(index: usize) -> Self {
        assert!(index < u8::MAX as usize - 1);
        G8torSemaphoreHandle {
            index: NonZeroU8::new((index as u8) + 1).unwrap(),
        }
    }
}

impl Into<usize> for G8torSemaphoreHandle {
    #[inline(always)]
    fn into(self) -> usize {
        (self.index.get() - 1) as usize
    }
}

pub(super) union G8torAtomic {
    pub(super) count: ManuallyDrop<AtomicU8>,
    pub(super) mutex: ManuallyDrop<AtomicBool>,
}

impl G8torAtomic {
    /// Create a new Semaphore atomic variable initialized to `initial`.
    #[inline(always)]
    pub const fn new_semaphore(initial: u8) -> Self {
        G8torAtomic {
            count: ManuallyDrop::new(AtomicU8::new(initial)),
        }
    }

    /// Create a new Mutex atomic variable initialized to unlocked (true).
    #[inline(always)]
    pub const fn new_mutex() -> Self {
        G8torAtomic {
            mutex: ManuallyDrop::new(AtomicBool::new(true)),
        }
    }
}

impl PartialEq<G8torSemaphoreHandle> for G8torAtomicHandle {
    fn eq(&self, other: &G8torSemaphoreHandle) -> bool {
        other.index == self.index
    }
}

impl PartialEq<G8torMutexHandle> for G8torAtomicHandle {
    fn eq(&self, other: &G8torMutexHandle) -> bool {
        other.index == self.index
    }
}
