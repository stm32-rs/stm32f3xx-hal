use crate::cubemx::ip::gpio;
use anyhow::{Context, Result};
use once_cell::sync::Lazy;
use regex::Regex;
use std::collections::HashMap;

struct Port<'a> {
    id: char,
    pins: Vec<&'a gpio::Pin>,
}

pub fn gen_mappings(gpio_ips: &[gpio::Ip]) -> Result<()> {
    for ip in gpio_ips.iter() {
        println!();
        gen_gpio_ip(ip)?;
    }
    Ok(())
}

fn gen_gpio_ip(ip: &gpio::Ip) -> Result<()> {
    let feature = ip_version_to_feature(&ip.version)?;
    let ports = merge_pins_by_port(&ip.pins)?;

    gen_gpio_macro_call(&ports, &feature)?;
    Ok(())
}

fn ip_version_to_feature(ip_version: &str) -> Result<String> {
    static VERSION: Lazy<Regex> =
        Lazy::new(|| Regex::new(r"^STM32(?P<version>\w+)_gpio_v1_0$").unwrap());

    let captures = VERSION
        .captures(ip_version)
        .with_context(|| format!("invalid GPIO IP version: {}", ip_version))?;

    let version = captures.name("version").unwrap().as_str();
    let feature = format!("gpio-{}", version.to_lowercase());
    Ok(feature)
}

fn merge_pins_by_port(pins: &[gpio::Pin]) -> Result<Vec<Port>> {
    let mut pins_by_port = HashMap::new();
    for pin in pins.iter() {
        pins_by_port
            .entry(pin.port()?)
            .and_modify(|e: &mut Vec<_>| e.push(pin))
            .or_insert_with(|| vec![pin]);
    }

    let mut ports = Vec::new();
    for (id, mut pins) in pins_by_port {
        pins.retain(|p| {
            p.name != "PDR_ON" && p.name != "PC14OSC32_IN" && p.name != "PC15OSC32_OUT"
        });
        pins.sort_by_key(|p| p.number().unwrap_or_default());
        pins.dedup_by_key(|p| p.number().unwrap_or_default());
        ports.push(Port { id, pins });
    }
    ports.sort_by_key(|p| p.id);

    Ok(ports)
}

fn gen_gpio_macro_call(ports: &[Port], feature: &str) -> Result<()> {
    for port in ports {
        println!(r#"#[cfg(feature = "{}")]"#, feature);
        gen_port(port)?;
        println!();
    }

    Ok(())
}

fn gen_port(port: &Port) -> Result<()> {
    let port_upper = port.id;
    let port_lower = port.id.to_ascii_lowercase();
    println!(
        "gpio!(GPIO{0}, gpio{1}, P{0}, '{0}', P{0}n, [",
        port_upper, port_lower
    );

    for pin in &port.pins {
        gen_pin(port_upper, port_lower, pin)?;
    }

    println!("]);");
    Ok(())
}

fn gen_pin(port_upper: char, port_lower: char, pin: &gpio::Pin) -> Result<()> {
    let nr = pin.number()?;
    let reset_mode = get_pin_reset_mode(pin)?;
    let af_numbers = get_pin_af_numbers(pin)?;

    println!(
        "    P{0}{2}: (p{1}{2}, {2}, {3:?}{4}),",
        port_upper,
        port_lower,
        nr,
        af_numbers,
        if let Some(rst) = reset_mode {
            format!(", {}", rst)
        } else {
            String::new()
        }
    );

    Ok(())
}

fn get_pin_reset_mode(pin: &gpio::Pin) -> Result<Option<&'static str>> {
    // Debug pins default to their debug function (AF0), everything else
    // defaults to floating input or analog.
    let mode = match (pin.port()?, pin.number()?) {
        ('A', 13) | ('A', 14) | ('A', 15) | ('B', 3) | ('B', 4) => Some("super::Debugger"),
        _ => None,
    };
    Ok(mode)
}

fn get_pin_af_numbers(pin: &gpio::Pin) -> Result<Vec<u8>> {
    let mut numbers = Vec::new();
    for signal in &pin.pin_signals {
        numbers.push(signal.af()?);
    }

    numbers.sort_unstable();
    numbers.dedup();

    Ok(numbers)
}
