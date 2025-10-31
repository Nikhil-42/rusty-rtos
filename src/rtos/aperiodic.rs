use cortex_m::peripheral::SCB;
use core::{mem::MaybeUninit, ptr};

pub(super) const NUM_EXCEPTIONS: usize = 16;    
pub(super) const NUM_IRQ: usize = 139;
const NUM_VECTORS: usize = NUM_EXCEPTIONS + NUM_IRQ;

extern "C" {
    static __vector_table: [unsafe extern "C" fn(); NUM_VECTORS];
}

#[link_section = ".vector_ram"]
static mut __VECTOR_RAM: MaybeUninit<[unsafe extern "C" fn(); NUM_VECTORS]> = MaybeUninit::uninit();

pub(super) unsafe fn relocate_vtor(scb: &mut SCB) {
    let __vector_table_ptr = __vector_table.as_ptr();
    let vector_ram_ptr = &raw mut __VECTOR_RAM as *mut _;
    ptr::write(vector_ram_ptr as *mut u32, 0x2000_8000);
    for i in 1..NUM_VECTORS {
        ptr::copy_nonoverlapping(__vector_table_ptr.add(i), (vector_ram_ptr as *mut unsafe extern "C" fn()).add(i), 1);
    }
    scb.vtor.write(vector_ram_ptr as u32);
}

pub(super) fn register_interrupt_handler(
    vector_number: usize,
    handler: extern "C" fn(),
) {
    let vector_ram_ptr = &raw mut __VECTOR_RAM as *mut _;
    unsafe {
        ptr::write(
            (vector_ram_ptr as *mut unsafe extern "C" fn()).add(vector_number + NUM_EXCEPTIONS),
            handler,
        );
    }
}
