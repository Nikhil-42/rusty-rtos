#![no_std]
#![no_main]

use core::ops::Deref;
pub mod rtos;

#[repr(transparent)]
pub struct SyncOnceCell<T> {
    value: core::cell::OnceCell<T>
}

impl<T> SyncOnceCell<T> {
    pub const fn new() -> Self {
        SyncOnceCell {
            value: core::cell::OnceCell::new()
        }
    }
}

impl<T> Deref for SyncOnceCell<T> {
    type Target = core::cell::OnceCell<T>;

    fn deref(&self) -> &Self::Target {
        &self.value
    }
}

unsafe impl<T> Sync for SyncOnceCell<T> {}