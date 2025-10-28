use core::{marker::PhantomData, ptr::NonNull};

pub(super) struct PeriodicTCB {
    pub period: u32,
    pub execution_time: u32,
    pub task: extern "C" fn(),
    pub next: NonNull<PeriodicTCB>,
    pub prev: NonNull<PeriodicTCB>,
}

impl<'a> IntoIterator for &'a mut PeriodicTCB {
    type Item = &'a mut PeriodicTCB;
    type IntoIter = PeriodicTCBIterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        PeriodicTCBIterator {
            end: self.prev,
            current: Some(NonNull::from(&*self)),
            _marker: PhantomData,
        }
    }
}

pub(super) struct PeriodicTCBIterator<'a> {
    end: NonNull<PeriodicTCB>,
    current: Option<NonNull<PeriodicTCB>>,
    _marker: PhantomData<&'a mut PeriodicTCB>,
}

impl<'a> Iterator for PeriodicTCBIterator<'a> {
    type Item = &'a mut PeriodicTCB;

    fn next(&mut self) -> Option<&'a mut PeriodicTCB> {
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


// SAFETY: executes in SysTick context which is uninterruptible
pub(super) fn _run_periodics(rtos: &'static mut super::G8torRtos) {
    if let Some(ptcb) = rtos.periodic.iter_mut().find_map(|p| p.as_mut()) {
        for ptcb in ptcb.into_iter() {
            if rtos.system_time == ptcb.execution_time {
                (ptcb.task)();
            }
        }
    }

}