use anyhow::{Context, Result};
use serde::Deserialize;
use std::{
    fs::File,
    path::{Path, PathBuf},
};

pub struct Db {
    root: PathBuf,
}

impl Db {
    pub fn new<P: Into<PathBuf>>(root: P) -> Self {
        Self { root: root.into() }
    }

    pub fn load<'de, P: AsRef<Path>, T: Deserialize<'de>>(&self, name: P) -> Result<T> {
        let name = name.as_ref();
        let mut path = self.root.join(name);
        path.set_extension("xml");

        let file = File::open(&path).with_context(|| format!("cannot open DB file: {:?}", path))?;
        serde_xml_rs::de::from_reader(file)
            .with_context(|| format!("cannot parse DB file: {:?}", path))
    }

    pub fn load_mcu<'de, P: AsRef<Path>, T: Deserialize<'de>>(&self, name: P) -> Result<T> {
        let mut mcu_path = PathBuf::new();
        mcu_path.push("mcu");
        mcu_path.push(name);
        self.load(&mcu_path)
    }
}
