#[test]
fn tests() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/gpio_pin_partially_erased_bad_pin.rs");
    t.compile_fail("tests/gpio_pin_accidental_erased.rs");
    t.compile_fail("tests/gpio_pin_forgot_af.rs")
}
