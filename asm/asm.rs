#![feature(asm)]
#![no_std]
#![crate_type = "staticlib"]

/// Perform volatile atomic `*ptr = *ptr & !bic | or;`
#[no_mangle]
pub unsafe extern "C" fn volatile_atomic_bic_or(ptr: *mut u32, bic: u32, or: u32) {
    let mut res: u32;
    while {
        asm!(
            "ldrex {0}, [{1}]",
            "bics {0}, {2}",
            "orrs {0}, {3}",
            "strex {0}, {0}, [{1}]",
            out(reg) res,
            in(reg) ptr,
            in(reg) bic,
            in(reg) or,
            options(nostack),
        );
        res != 0
    } {}
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {} // Now empty loop is safe for LLVM >=12!
}
