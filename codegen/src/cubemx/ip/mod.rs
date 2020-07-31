pub mod gpio;

use std::path::PathBuf;

fn ip_path(name: &str) -> PathBuf {
    ["IP", name].iter().collect()
}
