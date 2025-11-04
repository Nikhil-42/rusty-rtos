#![no_std]
#![no_main]

use core::{cell::UnsafeCell, mem::MaybeUninit, ops::Deref};
pub mod graphics;
pub mod physics;
pub mod rtos;

#[repr(transparent)]
pub struct SyncUnsafeOnceCell<T> {
    value: UnsafeCell<MaybeUninit<T>>,
}

impl<T> SyncUnsafeOnceCell<T> {
    pub const fn new() -> Self {
        SyncUnsafeOnceCell {
            value: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    pub unsafe fn set(&self, value: T) {
        (*self.value.get()).as_mut_ptr().write(value)
    }
}

impl<T> Deref for SyncUnsafeOnceCell<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { (*self.value.get()).assume_init_ref() }
    }
}

unsafe impl<T> Sync for SyncUnsafeOnceCell<T> {}
