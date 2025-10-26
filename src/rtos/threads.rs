use core::{marker::PhantomData, ptr::NonNull};
use crate::rtos::{G8torAtomicHandle};


#[repr(C)]
pub(super) struct TCB<const NAME_LEN: usize> {
    pub id: usize,
    pub sp: NonNull<u32>, // Stack pointer (points to top of stack)
    pub next: NonNull<TCB<NAME_LEN>>,
    pub prev: NonNull<TCB<NAME_LEN>>,
    pub sleep_until: u32,
    pub asleep: bool,
    pub priority: u8,
    pub blocked_by: Option<G8torAtomicHandle>, // A handle to the blocking atomic (if any)
    pub name: [u8; NAME_LEN],
}

impl<'a, const NAME_LEN: usize> IntoIterator for &'a mut TCB<NAME_LEN> {
    type Item = &'a mut TCB<NAME_LEN>;
    type IntoIter = TCBIterator<'a, NAME_LEN>;

    fn into_iter(self) -> Self::IntoIter {
        TCBIterator {
            end: self.prev,
            current: Some(NonNull::from(&*self)),
            _marker: PhantomData,
        }
    }
}

pub(super) struct TCBIterator<'a, const NAME_LEN: usize> {
    end: NonNull<TCB<NAME_LEN>>,
    current: Option<NonNull<TCB<NAME_LEN>>>,
    _marker: PhantomData<&'a mut TCB<NAME_LEN>>,
}

impl<'a, const NAME_LEN: usize> Iterator for TCBIterator<'a, NAME_LEN> {
    type Item = &'a mut TCB<NAME_LEN>;

    fn next(&mut self) -> Option<&'a mut TCB<NAME_LEN>> {
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
