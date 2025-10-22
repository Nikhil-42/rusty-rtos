use core::{cell::UnsafeCell, mem::ManuallyDrop, num::NonZeroU8, sync::atomic::{AtomicBool, AtomicU8}, u8};


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

impl<'a, T> From<G8torMutexHandle<'a, T>> for G8torAtomicHandle {
    #[inline(always)]
    fn from(mutex: G8torMutexHandle<'a, T>) -> Self {
        G8torAtomicHandle { index: mutex.index }
    }
}

#[derive(Clone, Copy)]
pub struct G8torMutexHandle<'a, T> {
    pub(super) index: NonZeroU8,
    pub(super) resource: &'a UnsafeCell<T>
}


impl<'a, T> Into<usize> for G8torMutexHandle<'a, T> {
    #[inline(always)]
    fn into(self) -> usize {
        (self.index.get() - 1) as usize
    }
}

pub struct G8torMutexLock {}

#[derive(Clone, Copy)]
pub struct G8torSemaphoreHandle {
    pub(super) index: NonZeroU8,
    pub(super) max_count: u8,
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

impl<'a, T> PartialEq<G8torMutexHandle<'a, T>> for G8torAtomicHandle {
    fn eq(&self, other: &G8torMutexHandle<'a, T>) -> bool {
        other.index == self.index
    }
}
