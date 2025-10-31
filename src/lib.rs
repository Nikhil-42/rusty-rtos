#![no_std]
#![no_main]

use core::{
    cell::UnsafeCell, mem::MaybeUninit, ops::{Deref, Index, IndexMut}, ptr::NonNull
};
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


struct StaticLinkedList<T, const N: usize> {
    nodes: [Option<StaticLinkedListNode<T>>; N],
}

struct StaticLinkedListNode<T> {
    data: T,
    next: NonNull<StaticLinkedListNode<T>>,
    prev: NonNull<StaticLinkedListNode<T>>,
}

impl<T, const N: usize> StaticLinkedList<T, N> {
    pub const fn empty() -> Self {
        StaticLinkedList {
            nodes: [const { None }; N],
        }
    }

    pub fn get(&self, index: usize) -> Option<&T> {
        self.nodes[index].as_ref().map(|node| &node.data)
    }

    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        self.nodes[index].as_mut().map(|node| &mut node.data)
    }
}

impl<T, const N: usize> Index<usize> for StaticLinkedList<T, N> {
    type Output = Option<StaticLinkedListNode<T>>;
    fn index(&self, index: usize) -> &Self::Output {
        &self.nodes[index]
    }
}

impl<T, const N: usize> IndexMut<usize> for StaticLinkedList<T, N> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.nodes[index]
    }
}
