use std::{
    fs,
    path::{Path, PathBuf},
};

use eframe::egui::{InputState, Key, Modifiers};
use once_cell::sync::OnceCell;
use serde::Deserialize;

use color_eyre::{
    Result,
    eyre::{Context, bail},
};

pub static CONFIG: OnceCell<Config> = OnceCell::new();

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct Config {
    pub keybindings: KeyBindings,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            keybindings: KeyBindings::default(),
        }
    }
}

pub fn load_config(explicit_path: Option<&Path>) -> Result<Config> {
    let Some(path) = explicit_path
        .map(Path::to_path_buf)
        .or_else(default_config_path)
    else {
        return Ok(Config::default());
    };

    if !path.exists() {
        if explicit_path.is_some() {
            bail!("config file does not exist: {}", path.display());
        }
        return Ok(Config::default());
    }

    let config = fs::read_to_string(&path)
        .wrap_err_with(|| format!("failed to read config file {}", path.display()))?;
    toml::from_str(&config)
        .wrap_err_with(|| format!("failed to parse config file {}", path.display()))
}

fn default_config_path() -> Option<PathBuf> {
    xdg_config_path_from(
        std::env::var_os("XDG_CONFIG_HOME").map(PathBuf::from),
        std::env::var_os("HOME").map(PathBuf::from),
    )
}

pub fn xdg_config_path_from(
    xdg_config_home: Option<PathBuf>,
    home: Option<PathBuf>,
) -> Option<PathBuf> {
    xdg_config_home
        .or_else(|| home.map(|home| home.join(".config")))
        .map(|config_home| config_home.join("annotato").join("config.toml"))
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct KeyBindings {
    pub next: KeyBind,
    pub previous: KeyBind,
    pub save: KeyBind,
    pub edit: KeyBind,
    pub draw: KeyBind,
    pub move_box: KeyBind,
    pub cycle_corner: KeyBind,
    pub confirm: KeyBind,
    pub abort: KeyBind,
    pub delete: KeyBind,
    pub next_class: KeyBind,
    pub previous_class: KeyBind,

    pub select_ball: Key,
    pub select_robot: Key,
    pub select_goalpost: Key,
    pub select_penaltyspot: Key,
    pub select_xspot: Key,
    pub select_lspot: Key,
    pub select_tspot: Key,
    pub select_person: Key,
}

impl Default for KeyBindings {
    fn default() -> Self {
        Self {
            next: KeyBind::new(Key::Space),
            previous: KeyBind::new(Key::Space).with_modifiers(Modifiers::SHIFT),
            save: KeyBind::new(Key::S),
            edit: KeyBind::new(Key::E),
            draw: KeyBind::new(Key::B),
            move_box: KeyBind::new(Key::M),
            cycle_corner: KeyBind::new(Key::C),
            confirm: KeyBind::new(Key::Enter),
            abort: KeyBind::new(Key::Escape),
            delete: KeyBind::new(Key::Delete),
            next_class: KeyBind::new(Key::CloseBracket),
            previous_class: KeyBind::new(Key::OpenBracket),
            select_ball: Key::Num1,
            select_robot: Key::Num2,
            select_goalpost: Key::Num3,
            select_penaltyspot: Key::Num4,
            select_xspot: Key::Num5,
            select_lspot: Key::Num6,
            select_tspot: Key::Num7,
            select_person: Key::Num8,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct KeyBind {
    pub primary: Key,
    #[serde(alias = "extra")]
    pub alternatives: Vec<Key>,
    #[serde(deserialize_with = "deserialize_modifiers", default)]
    pub modifiers: Modifiers,
}

impl Default for KeyBind {
    fn default() -> Self {
        Self::new(Key::Space)
    }
}

impl KeyBind {
    pub fn new(primary: Key) -> Self {
        Self {
            primary,
            alternatives: Vec::new(),
            modifiers: Modifiers::default(),
        }
    }

    fn with_modifiers(mut self, modifiers: Modifiers) -> Self {
        self.modifiers = modifiers;
        self
    }

    pub fn is_pressed(&self, input: &InputState) -> bool {
        let modifiers_pressed = input.modifiers == self.modifiers;
        let keys_pressed = [self.primary]
            .iter()
            .chain(self.alternatives.iter())
            .any(|&key| input.key_pressed(key));

        modifiers_pressed && keys_pressed
    }
}

fn deserialize_modifiers<'de, D>(deserializer: D) -> Result<Modifiers, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let string_modifiers = Vec::<String>::deserialize(deserializer)?;
    let mut modifiers = Modifiers::default();
    for modifier in string_modifiers {
        match modifier.as_str() {
            "alt" => modifiers |= Modifiers::ALT,
            "ctrl" => modifiers |= Modifiers::CTRL,
            "shift" => modifiers |= Modifiers::SHIFT,
            invalid => {
                return Err(serde::de::Error::custom(format!(
                    "invalid modifier: {invalid}"
                )));
            }
        }
    }
    Ok(modifiers)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_path_prefers_xdg_config_home() {
        let path = xdg_config_path_from(
            Some(PathBuf::from("/xdg")),
            Some(PathBuf::from("/home/user")),
        )
        .unwrap();
        assert_eq!(path, PathBuf::from("/xdg/annotato/config.toml"));
    }

    #[test]
    fn config_path_falls_back_to_home_config() {
        let path = xdg_config_path_from(None, Some(PathBuf::from("/home/user"))).unwrap();
        assert_eq!(
            path,
            PathBuf::from("/home/user/.config/annotato/config.toml")
        );
    }
}
