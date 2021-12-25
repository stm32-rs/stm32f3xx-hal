//! Register modifying support

use core::sync::atomic::{compiler_fence, Ordering};

/// Modify specific index of array-like register
macro_rules! modify_at {
    ($reg:expr, $bitwidth:expr, $index:expr, $value:expr) => {
        $reg.modify(|r, w| {
            let mask = !(u32::MAX >> (32 - $bitwidth) << ($bitwidth * $index));
            let value = $value << ($bitwidth * $index);
            w.bits(r.bits() & mask | value)
        })
    };
}
pub(crate) use modify_at;

/// Modify specific index of array-like register atomically
///
/// # SAFETY
///
/// Ensure that the pointer points to valid memory address. Usually this is used for
/// peripheral registers.
#[inline]
pub(crate) unsafe fn atomic_modify_at(register: *mut u32, bitwidth: u8, index: u8, value: u32) {
    let mask_comparison = u32::MAX >> (32 - bitwidth) << (bitwidth * index);
    let value = value << (bitwidth * index);

    // Perform volatile atomic `*reg = *reg & !mask_comp | value;`
    let mut result: u32;
    // Try the atomic operation until the store was successful.
    loop {
        compiler_fence(Ordering::SeqCst);
        // Load the memory location exclusively. A signal to the CPU that in no other context a load of this
        // memory address is possible.
        // See https://developer.arm.com/documentation/dui0489/i/arm-and-thumb-instructions/ldrex
        //
        // `result` contains the current content of the peripheral register before modification.
        core::arch::asm!(
            "ldrex {0}, [{1}]",
            out(reg) result,
            in(reg) register,
            options(readonly, preserves_flags, nostack),
        );
        compiler_fence(Ordering::SeqCst);
        // We have exclusive access to the memory address and can operate the "modify at"
        // operations (which seems atomic to the caller because of the exclusive load).
        result = result & !mask_comparison | value;
        compiler_fence(Ordering::SeqCst);
        // Get exclusive access to the memory register to store it
        // `result` contains success code, weither the store was successful or not.
        core::arch::asm!(
            "strex {0}, {0}, [{1}]",
            inout(reg) result,
            in(reg) register,
            options(preserves_flags, nostack),
        );
        compiler_fence(Ordering::SeqCst);
        // If the store was successful, 0 is returned, see
        // https://developer.arm.com/documentation/dui0489/i/arm-and-thumb-instructions/strex
        if result == 0 {
            break;
        }
    }
}
