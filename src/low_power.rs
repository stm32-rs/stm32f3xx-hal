//! This module contains code used to place the STM32F3 in low power modes.
//! Reference section 3.7: `Power management` of the user manual,
//! and more importantly, 7.3: `Low-power modes` of reference manual.

/// Enter `Sleep now` mode: the lightest of the 3 low-power states avail on the
/// STM32f3.
/// TO exit: Interrupt. Refer to Table 82.
use crate::pac::PWR;
use cortex_m::{asm::wfi, peripheral::SCB};

/// Ref man, table 18.
pub fn sleep_now(scb: &mut SCB) {
    // WFI (Wait for Interrupt) (eg `cortext_m::asm::wfi()) or WFE (Wait for Event) while:

    // SLEEPDEEP = 0 and SLEEPONEXIT = 0
    scb.clear_sleepdeep();
    // Sleep-now: if the SLEEPONEXIT bit is cleared, the MCU enters Sleep mode as soon
    // as WFI or WFE instruction is executed.
    scb.clear_sleeponexit();

    wfi();
}

// Ref man, table 19.
pub fn sleep_on_exit(scb: &mut SCB) {
    // WFI (Wait for Interrupt) (eg `cortext_m::asm::wfi()) or WFE (Wait for Event) while:

    // SLEEPDEEP = 0 and SLEEPONEXIT = 0
    scb.clear_sleepdeep();
    // Sleep-now: if the SLEEPONEXIT bit is cleared, the MCU enters Sleep mode as soon
    // as WFI or WFE instruction is executed.
    scb.set_sleeponexit();

    wfi();
}

/// Enter `Stop` mode: the middle of the 3 low-power states avail on the
/// STM32f3.
/// To exit:  Any EXTI Line configured in Interrupt mode (the corresponding EXTI
/// Interrupt vector must be enabled in the NVIC). Refer to Table 82.
/// Ref man, table 20.
pub fn stop(scb: &mut SCB, pwr: &mut PWR) {
    //WFI (Wait for Interrupt) or WFE (Wait for Event) while:

    // Set SLEEPDEEP bit in ARM® Cortex®-M4 System Control register
    scb.set_sleepdeep();

    // Clear PDDS bit in Power Control register (PWR_CR)
    // This bit is set and cleared by software. It works together with the LPDS bit.
    // 0: Enter Stop mode when the CPU enters Deepsleep. The regulator status
    // depends on the LPDS bit.
    // 1: Enter Standby mode when the CPU enters Deepsleep.
    pwr.cr.modify(|_, w| w.pdds().clear_bit());

    // Select the voltage regulator mode by configuring LPDS bit in PWR_CR
    // This bit is set and cleared by software. It works together with the PDDS bit.
    // 0: Voltage regulator on during Stop mode
    // 1: Voltage regulator in low-power mode during Stop mode
    pwr.cr.modify(|_, w| w.lpds().set_bit());

    wfi();
}

/// Enter `Standby` mode: the lowest-power of the 3 low-power states avail on the
/// STM32f3.
/// To exit: WKUP pin rising edge, RTC alarm event’s rising edge, external Reset in
/// NRST pin, IWDG Reset.
/// Ref man, table 21.
pub fn standby(scb: &mut SCB, pwr: &mut PWR) {
    // WFI (Wait for Interrupt) or WFE (Wait for Event) while:

    // Set SLEEPDEEP bit in ARM® Cortex®-M4 System Control register
    scb.set_sleepdeep();

    // Set PDDS bit in Power Control register (PWR_CR)
    // This bit is set and cleared by software. It works together with the LPDS bit.
    // 0: Enter Stop mode when the CPU enters Deepsleep. The regulator status
    // depends on the LPDS bit.
    // 1: Enter Standby mode when the CPU enters Deepsleep.
    pwr.cr.modify(|_, w| w.pdds().set_bit());

    // Clear WUF bit in Power Control/Status register (PWR_CSR) (Must do this by setting CWUF bit in
    // PWR_CR.)
    pwr.cr.modify(|_, w| w.cwuf().set_bit());

    wfi();
}
