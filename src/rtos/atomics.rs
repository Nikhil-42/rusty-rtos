use core::{cell::UnsafeCell, mem::MaybeUninit, num::NonZeroU8, u8};


#[derive(Clone, Copy, Eq, PartialEq)]
pub(super) struct G8torAtomicHandle {
    pub(super) indexp1: NonZeroU8,
}

impl G8torAtomicHandle {
    #[inline(always)]
    pub const fn from_index(index: u8) -> Self {
        G8torAtomicHandle { indexp1: NonZeroU8::new(index + 1).unwrap() }
    }
}

impl From<G8torSemaphoreHandle> for G8torAtomicHandle {
    #[inline(always)]
    fn from(sem: G8torSemaphoreHandle) -> Self {
        Self::from_index(sem.index)
    }
}

impl<'a, T> From<G8torMutexHandle<T>> for G8torAtomicHandle {
    #[inline(always)]
    fn from(mutex: G8torMutexHandle<T>) -> Self {
        Self::from_index(mutex.index)
    }
}

#[derive(Clone, Copy)]
pub struct G8torMutexHandle<T: 'static> {
    pub(super) index: u8,
    pub(super) mutex: &'static G8torMutex<T>,
}

pub struct G8torMutexLock<T: 'static> {
    pub(super) mutex: &'static G8torMutex<T>,
}

#[derive(Clone, Copy)]
pub struct G8torSemaphoreHandle {
    pub(super) index: u8,
    pub(super) max_count: u8,
}

impl PartialEq<G8torSemaphoreHandle> for G8torAtomicHandle {
    fn eq(&self, other: &G8torSemaphoreHandle) -> bool {
        other.index == self.indexp1.get()
    }
}

impl<'a, T> PartialEq<G8torMutexHandle<T>> for G8torAtomicHandle {
    fn eq(&self, other: &G8torMutexHandle<T>) -> bool {
        other.index == self.indexp1.get()
    }
}

unsafe impl<T> Sync for G8torMutex<T> {}

#[repr(transparent)]
pub struct G8torMutex<T> {
    resource: UnsafeCell<MaybeUninit<T>>,
}

impl<T> G8torMutex<T> {
    pub const fn empty() -> Self
    {
        G8torMutex {
            resource: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    /// Unsafe: Must only be called once to initialize the mutex resource
    pub unsafe fn init(&'static self, resource: T) {
        let res_ptr = self.resource.get();
        res_ptr.write(MaybeUninit::new(resource));
    }

    /// Trade the lock for the resource
    pub fn get(&'static self, lock: G8torMutexLock<T>) -> &'static mut T {
        if core::ptr::eq(self, lock.mutex) {
            // SAFETY: The lock guarantees exclusive access to the resource.
            unsafe {
                (*self.resource.get()).assume_init_mut()
            }
        }
        else {
            panic!("Attempted to get a mutex resource with an invalid lock.");
        }
    }

    /// Trade the resource back for a lock
    pub fn release(&'static self, resource: &'static mut T) -> G8torMutexLock<T> {
        if core::ptr::eq(unsafe { (*self.resource.get()).as_ptr() }, resource) {
            // Resource released successfully
            return G8torMutexLock { mutex: self };
        }
        else {
            panic!("Attempted to release a mutex with an invalid resource reference.");
        }
    }
}