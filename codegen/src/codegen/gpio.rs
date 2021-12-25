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

    println!(r#"#[cfg(feature = "{}")]"#, feature);
    gen_gpio_macro_call(&ports, &feature)?;
    Ok(())
}

fn ip_version_to_feature(ip_version: &str) -> Result<String> {
    static VERSION: Lazy<Regex> =
        Lazy::new(|| Regex::new(r"^STM32(?P<version>\w+)_gpio_v1_0$").unwrap());

    let captures = VERSION
        .captures(&ip_version)
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
        pins.sort_by_key(|p| p.number().unwrap_or_default());
        pins.dedup_by_key(|p| p.number().unwrap_or_default());
        ports.push(Port { id, pins });
    }
    ports.sort_by_key(|p| p.id);

    Ok(ports)
}

fn gen_gpio_macro_call(ports: &[Port], feature: &str) -> Result<()> {
    println!("gpio!({{");

    gen_pac_list(ports, feature);

    println!("    ports: [");
    for port in ports {
        gen_port(port, feature)?;
    }
    println!("    ],");

    println!("}});");
    Ok(())
}

fn gen_pac_list(ports: &[Port], feature: &str) {
    let mut pac_modules: Vec<_> = ports
        .iter()
        .map(|port| get_port_pac_module(port, feature))
        .collect();
    pac_modules.sort_unstable();
    pac_modules.dedup();
    println!("    pacs: [{}],", pac_modules.join(", "));
}

fn gen_port(port: &Port, feature: &str) -> Result<()> {
    let pac_module = get_port_pac_module(port, feature);
    let port_index = match port.id {
        'A' => 0,
        'B' => 1,
        'C' => 2,
        'D' => 3,
        'E' => 4,
        'F' => 5,
        'G' => 6,
        'H' => 7,
        _ => unreachable!(),
    };

    println!("        {{");
    println!(
        "            port: ({}/{}, {}, {}),",
        port.id,
        port.id.to_lowercase(),
        port_index,
        pac_module,
    );
    println!("            pins: [");

    for pin in &port.pins {
        gen_pin(pin)?;
    }

    println!("            ],");
    println!("        }},");
    Ok(())
}

fn get_port_pac_module(port: &Port, feature: &str) -> &'static str {
    // The registers in ports A and B have different reset values due to the
    // presence of debug pins, so they get dedicated PAC modules.
    match port.id {
        'A' => "gpioa",
        'B' => "gpiob",
        'D' if feature == "gpio-f373" => "gpiod",
        _ => "gpioc",
    }
}

fn gen_pin(pin: &gpio::Pin) -> Result<()> {
    let nr = pin.number()?;
    let reset_mode = get_pin_reset_mode(pin)?;
    let af_numbers = get_pin_af_numbers(pin)?;

    println!(
        "                {} => {{ reset: {}, af: {:?} }},",
        nr, reset_mode, af_numbers,
    );

    Ok(())
}

fn get_pin_reset_mode(pin: &gpio::Pin) -> Result<&'static str> {
    // Debug pins default to their debug function (AF0), everything else
    // defaults to input.
    let mode = match (pin.port()?, pin.number()?) {
        ('A', 13) | ('A', 14) | ('A', 15) | ('B', 3) | ('B', 4) => "AF0<PushPull>",
        _ => "Input",
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
