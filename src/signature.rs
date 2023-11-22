//! Device electronic signature
//!
//! (stored in flash memory)
use core::fmt;
use core::str;

const UID_PTR: u32 = 0x1FFF_F7AC;

use core::convert::TryInto;

impl Uid {
    fn ptr() -> *const Self {
        UID_PTR as *const _
    }

    /// Returns a wrapped reference to the value in flash memory
    #[must_use]
    pub fn get() -> &'static Self {
        // SAFETY: The Uid pointer is definitly correct and as it is only ever shared immutable
        // mutliple references to it are fine.
        unsafe { &*Self::ptr() }
    }
}

/// Uniqure Device ID register
#[repr(C, packed)]
pub struct Uid {
    x: u16,
    y: u16,
    waf: u8,
    lot: [u8; 7],
}

#[cfg(feature = "defmt")]
impl defmt::Format for Uid {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Peripheral {{ x: {:x}, y: {:x}, waf: {}, lum: {}}}",
            { self.x },
            { self.y },
            { self.waf },
            { self.lot_number() },
        );
    }
}

impl fmt::Debug for Uid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Uid")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("waf", &self.waf)
            .field("lot", &self.lot_number())
            .finish()
    }
}

fn bcd_to_num(bcd_num: u16) -> u16 {
    bcd_num
        .to_ne_bytes()
        .iter()
        .enumerate()
        .map(|(i, byte)| (i * 2, (*byte & 0xF0) >> 4, *byte & 0x0F))
        .map(|(i, high_nibble, low_nibble)| {
            let i: u32 = i.try_into().unwrap_or_default(); // This should never overflow
            u16::from(high_nibble) * 10u16.pow(i + 1) + u16::from(low_nibble) * 10u16.pow(i)
        })
        .sum()
}

impl Uid {
    /// X coordinate on wafer in BCD format
    #[must_use]
    pub fn x_bcd(&self) -> u16 {
        self.x
    }

    /// X coordinate on wafer
    #[must_use]
    pub fn x(&self) -> u16 {
        bcd_to_num(self.x)
    }

    /// Y coordinate on wafer in BCD format
    #[must_use]
    pub fn y_bcd(&self) -> u16 {
        self.y
    }

    /// Y coordinate on wafer
    #[must_use]
    pub fn y(&self) -> u16 {
        bcd_to_num(self.y)
    }

    /// Wafer number
    #[must_use]
    pub fn wafer_number(&self) -> u8 {
        self.waf
    }

    /// Lot number
    #[must_use]
    pub fn lot_number(&self) -> &str {
        // Lets ignore the last byte, because it is a '\0' character.
        // TODO: change to core::ffi::CStr
        // SAFETY: It is assumed that the lot number only contains valid ASCII characters
        unsafe { str::from_utf8_unchecked(&self.lot[..6]) }
    }
}

/// Size of integrated flash
#[derive(Debug)]
#[repr(C)]
pub struct FlashSize(u16);

const FLASH_PTR: u32 = 0x1FFF_F7CC;
impl FlashSize {
    fn ptr() -> *const Self {
        FLASH_PTR as *const _
    }

    /// Returns a wrapped reference to the value in flash memory
    #[must_use]
    pub fn get() -> &'static Self {
        // SAFETY: The FlashSize pointer is definitly correct and as it is only ever shared immutable
        // mutliple references to it are fine.
        unsafe { &*Self::ptr() }
    }
}

impl FlashSize {
    /// Read flash size in kilobytes
    #[must_use]
    pub fn kilo_bytes(&self) -> u16 {
        self.0
    }

    /// Read flash size in bytes
    #[must_use]
    pub fn bytes(&self) -> usize {
        usize::from(self.kilo_bytes()) * 1024
    }
}
