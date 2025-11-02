use core::{marker::PhantomData, ptr::NonNull};
use crate::rtos::{G8torAtomicHandle};


#[repr(C)]
pub(super) struct TCB {
    pub id: usize,
    pub sp: NonNull<u32>, // Stack pointer (points to top of stack)
    pub next: NonNull<TCB>,
    pub prev: NonNull<TCB>,
    pub sleep_until: u32,
    pub asleep: bool,
    pub priority: u8,
    pub blocked_by: Option<G8torAtomicHandle>, // A handle to the blocking atomic (if any)
    pub lr: u8, // Link register for context switching
    pub name: [u8; 16],
}

impl<'a> IntoIterator for &'a mut TCB {
    type Item = &'a mut TCB;
    type IntoIter = TCBIterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        TCBIterator {
            end: self.prev,
            current: Some(NonNull::from(&*self)),
            _marker: PhantomData,
        }
    }
}

pub(super) struct TCBIterator<'a> {
    end: NonNull<TCB>,
    current: Option<NonNull<TCB>>,
    _marker: PhantomData<&'a mut TCB>,
}

impl<'a> Iterator for TCBIterator<'a> {
    type Item = &'a mut TCB;

    fn next(&mut self) -> Option<&'a mut TCB> {
        let mut curr = self.current?;
        let next= unsafe { curr.as_ref() }.next;
        if curr == self.end {
            self.current = None;
        } else {
            self.current = Some(next);
        }
        // Move to the next TCB
        Some(unsafe { curr.as_mut() } )
    }
}
