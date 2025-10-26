
pub struct G8torFifoHandle {
    pub(super) index: u8,
}

pub(super) struct G8torFIFO<const LEN: usize> {
    head: usize,
    tail: usize,
    lost: usize,
    buffer: [u32; LEN],
}