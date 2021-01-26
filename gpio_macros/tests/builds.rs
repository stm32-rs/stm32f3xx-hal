#[test]
fn tests() {
    let t = trybuild::TestCases::new();
    t.pass("tests/gpio_pin_partially_erased.rs");
    t.compile_fail("tests/gpio_pin_partially_erased_bad_pin.rs");
}
