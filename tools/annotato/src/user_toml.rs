use std::{
    fmt::Write as _,
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
#[serde(default, deny_unknown_fields)]
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
    parse_config(&config)
        .wrap_err_with(|| format!("failed to parse config file {}", path.display()))
}

fn parse_config(config: &str) -> std::result::Result<Config, toml::de::Error> {
    let mut config = toml::from_str::<toml::Value>(config)?;
    remove_legacy_keybindings(&mut config);
    config.try_into()
}

fn remove_legacy_keybindings(config: &mut toml::Value) {
    let Some(keybindings) = config
        .get_mut("keybindings")
        .and_then(toml::Value::as_table_mut)
    else {
        return;
    };

    for legacy_key in ["cycle_corner", "next_class", "previous_class"] {
        keybindings.remove(legacy_key);
    }
    keybindings.retain(|key, _| !key.starts_with("select_"));
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
#[serde(default, deny_unknown_fields)]
pub struct KeyBindings {
    pub next: KeyBind,
    pub previous: KeyBind,
    pub save: KeyBind,
    pub class_popup: KeyBind,
    pub temporary_focus: KeyBind,
    pub edit: KeyBind,
    pub draw: KeyBind,
    pub move_box: KeyBind,
    pub confirm: KeyBind,
    pub abort: KeyBind,
    pub delete: KeyBind,
}

impl Default for KeyBindings {
    fn default() -> Self {
        Self {
            next: KeyBind::new(Key::Space),
            previous: KeyBind::new(Key::Space).with_modifiers(Modifiers::SHIFT),
            save: KeyBind::new(Key::S),
            class_popup: KeyBind::new(Key::C),
            temporary_focus: KeyBind::new(Key::F),
            edit: KeyBind::new(Key::E),
            draw: KeyBind::new(Key::B),
            move_box: KeyBind::new(Key::M),
            confirm: KeyBind::new(Key::Enter),
            abort: KeyBind::new(Key::Escape),
            delete: KeyBind::new(Key::Delete),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct KeyBind {
    pub primary: Key,
    #[serde(alias = "extra")]
    pub alternatives: Vec<Key>,
    #[serde(deserialize_with = "deserialize_modifiers", default)]
    pub modifiers: Modifiers,
    #[serde(skip)]
    label: OnceCell<String>,
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
            label: OnceCell::new(),
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

    pub fn is_down(&self, input: &InputState) -> bool {
        let modifiers_pressed = input.modifiers == self.modifiers;
        let keys_down = [self.primary]
            .iter()
            .chain(self.alternatives.iter())
            .any(|&key| input.key_down(key));

        modifiers_pressed && keys_down
    }

    pub fn label(&self) -> &str {
        self.label.get_or_init(|| {
            let mut label = String::new();
            push_modifier_label(&mut label, self.modifiers);
            if !label.is_empty() {
                label.push('+');
            }
            write!(&mut label, "{:?}", self.primary).expect("writing to String failed");
            for alternative in &self.alternatives {
                label.push('/');
                write!(&mut label, "{alternative:?}").expect("writing to String failed");
            }
            label
        })
    }
}

fn push_modifier_label(label: &mut String, modifiers: Modifiers) {
    let mut separator = "";
    if modifiers.ctrl {
        label.push_str(separator);
        label.push_str("Ctrl");
        separator = "+";
    }
    if modifiers.alt {
        label.push_str(separator);
        label.push_str("Alt");
        separator = "+";
    }
    if modifiers.shift {
        label.push_str(separator);
        label.push_str("Shift");
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

    #[test]
    fn default_shortcuts_include_class_popup_and_focus() {
        let keybindings = KeyBindings::default();

        assert_eq!(keybindings.class_popup.primary, Key::C);
        assert_eq!(keybindings.temporary_focus.primary, Key::F);
    }

    #[test]
    fn empty_config_uses_defaults() {
        let config = parse_config("").unwrap();

        assert_eq!(config.keybindings.class_popup.primary, Key::C);
    }

    #[test]
    fn unknown_config_fields_are_rejected() {
        let error = parse_config("unknown = true").unwrap_err();

        assert!(error.to_string().contains("unknown"));
    }

    #[test]
    fn legacy_keybindings_are_ignored() {
        let config = parse_config(
            r#"
            [keybindings]
            cycle_corner = true
            next_class = { primary = "Num1" }
            previous_class = { primary = "Num2" }
            select_robot = { primary = "R" }
            class_popup = { primary = "C" }
            "#,
        )
        .unwrap();

        assert_eq!(config.keybindings.class_popup.primary, Key::C);
    }

    #[test]
    fn unknown_keybinding_fields_are_rejected() {
        let error = parse_config("[keybindings]\nunknown = true").unwrap_err();

        assert!(error.to_string().contains("unknown"));
    }
}
