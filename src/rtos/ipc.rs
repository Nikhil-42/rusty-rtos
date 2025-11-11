use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicU32, Ordering},
};

use embedded_hal::i2c::{ErrorType, I2c};

use crate::rtos::{G8torSemaphoreHandle, FIFO_SIZE as LEN};

#[derive(Copy, Clone, Debug)]
pub struct G8torFifoHandle {
    pub(super) index: u8,
}

pub(super) struct G8torFifoInternals {
    head: AtomicU32,
    tail: AtomicU32,
    lost: AtomicU32,
    buffer: [UnsafeCell<u32>; LEN],
}

const MASK: u32 = const {
    assert!(LEN.is_power_of_two());
    (LEN as u32) - 1
};

impl G8torFifoInternals {
    pub fn write(&self, val: u32) -> bool {
        let old_tail = self.tail.fetch_add(1, Ordering::AcqRel);
        let idx = (old_tail & MASK) as usize;

        // Write data into buffer slot (unsafe cell)
        unsafe {
            self.buffer[idx].get().write(val);
        }

        // Now check whether we've advanced past head by more than capacity
        let head = self.head.load(Ordering::Acquire);
        let len = (old_tail.wrapping_add(1)).wrapping_sub(head); // number of elements after this push
        let lost = if len > (LEN as u32) {
            // buffer overflow — advance head to drop the oldest element
            self.head.fetch_add(1, Ordering::AcqRel);
            self.lost.fetch_add(1, Ordering::Relaxed);
            true
        } else {
            false
        };

        lost
    }

    /// Reads a value from the FIFO. Assumes that there is data available. Caller must ensure this.
    pub fn read(&self) -> u32 {
        let head = self.head.fetch_add(1, Ordering::AcqRel);
        let idx = (head & MASK) as usize;
        unsafe { self.buffer[idx].get().read() }
    }
}

pub(super) struct G8torFifo {
    pub(super) internals: G8torFifoInternals,
    pub(super) semaphore_handle: G8torSemaphoreHandle,
}

impl G8torFifo {
    pub const fn new(semaphore_handle: G8torSemaphoreHandle) -> Self {
        G8torFifo {
            internals: G8torFifoInternals {
                head: AtomicU32::new(0),
                tail: AtomicU32::new(0),
                lost: AtomicU32::new(0),
                buffer: [const { UnsafeCell::new(0) }; LEN],
            },
            semaphore_handle,
        }
    }

    pub fn read(&'static self) -> u32 {
        self.internals.read()
    }

    pub fn write(&'static self, val: u32) -> bool {
        self.internals.write(val)
    }
}

// Implement I2C for a Mutex that guards an I2C bus
impl<T: I2c> ErrorType for super::G8torMutexHandle<T> {
    type Error = T::Error;
}

impl<T: I2c> I2c for super::G8torMutexHandle<T> {
    #[inline]
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        let lock = super::take_mutex(self);
        let bus = self.mutex.get(lock);
        let res = bus.read(address, read);
        let lock = self.mutex.release(bus);
        super::release_mutex(&self, lock);
        res
    }

    #[inline]
    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        let lock = super::take_mutex(&self);
        let bus = self.mutex.get(lock);
        let res = bus.write(address, write);
        let lock = self.mutex.release(bus);
        super::release_mutex(&self, lock);
        res
    }

    #[inline]
    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let lock = super::take_mutex(&self);
        let bus = self.mutex.get(lock);
        let res = bus.write_read(address, write, read);
        let lock = self.mutex.release(bus);
        super::release_mutex(&self, lock);
        res
    }

    #[inline]
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let lock = super::take_mutex(&self);
        let bus = self.mutex.get(lock);
        let res = bus.transaction(address, operations);
        let lock = self.mutex.release(bus);
        super::release_mutex(&self, lock);
        res
    }
}

impl<D, T: eh0::serial::Read<D>> eh0::serial::Read<D> for &super::G8torMutexHandle<T> {
    type Error = T::Error;

    fn read(&mut self) -> Result<D, nb::Error<Self::Error>> {
        loop {
            let lock = super::take_mutex(&self);
            let bus = self.mutex.get(lock);
            let res = bus.read();
            let lock = self.mutex.release(bus);
            super::release_mutex(&self, lock);

            match res {
                Ok(v) => return Ok(v),
                Err(nb::Error::WouldBlock) => continue,
                Err(e) => return Err(e),
            }
        }
    }
}

impl<D: Copy, T: eh0::serial::Write<D>> eh0::serial::Write<D> for &super::G8torMutexHandle<T> {
    type Error = T::Error;

    fn write(&mut self, word: D) -> Result<(), nb::Error<Self::Error>> {
        loop {
            let lock = super::take_mutex(&self);
            let bus = self.mutex.get(lock);
            let res = bus.write(word);
            let lock = self.mutex.release(bus);
            super::release_mutex(&self, lock);

            match res {
                Err(nb::Error::WouldBlock) => continue,
                other => break other,
            }
        }
    }

    fn flush(&mut self) -> Result<(), nb::Error<Self::Error>> {
        loop {
            let lock = super::take_mutex(&self);
            let bus = self.mutex.get(lock);
            let res = bus.flush();
            let lock = self.mutex.release(bus);
            super::release_mutex(&self, lock);

            match res {
                Err(nb::Error::WouldBlock) => continue,
                other => break other,
            }
        }
    }
}