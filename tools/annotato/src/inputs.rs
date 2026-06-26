use std::{
    collections::BTreeSet,
    fs,
    path::{Path, PathBuf},
};

use color_eyre::{
    Result,
    eyre::{Context, bail},
};
use glob::glob;

const IMAGE_EXTENSIONS: &[&str] = &["png", "jpg", "jpeg", "bmp", "webp"];

pub fn collect_image_paths(inputs: &[String]) -> Result<Vec<PathBuf>> {
    let mut paths = BTreeSet::new();

    for input in inputs {
        if looks_like_glob(input) {
            for entry in glob(input).wrap_err_with(|| format!("invalid glob pattern {input:?}"))? {
                add_path(entry?, &mut paths)?;
            }
            continue;
        }

        let path = PathBuf::from(input);
        if !path.exists() {
            bail!("input does not exist: {}", path.display());
        }
        add_path(path, &mut paths)?;
    }

    if paths.is_empty() {
        bail!("no supported images found");
    }

    Ok(paths.into_iter().collect())
}

fn add_path(path: PathBuf, paths: &mut BTreeSet<PathBuf>) -> Result<()> {
    if path.is_dir() {
        for entry in
            fs::read_dir(&path).wrap_err_with(|| format!("failed to read {}", path.display()))?
        {
            let entry = entry?;
            let entry_path = entry.path();
            if entry_path.is_file() && is_image_path(&entry_path) {
                paths.insert(entry_path.to_path_buf());
            }
        }
    } else if path.is_file() && is_image_path(&path) {
        paths.insert(path);
    }

    Ok(())
}

fn looks_like_glob(input: &str) -> bool {
    input.contains('*') || input.contains('?') || input.contains('[')
}

pub fn is_image_path(path: &Path) -> bool {
    path.extension()
        .and_then(|extension| extension.to_str())
        .map(|extension| {
            IMAGE_EXTENSIONS
                .iter()
                .any(|supported| extension.eq_ignore_ascii_case(supported))
        })
        .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use std::{
        fs::{self, File},
        time::{SystemTime, UNIX_EPOCH},
    };

    use super::*;

    fn temp_dir() -> PathBuf {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!("annotato-inputs-{suffix}"));
        fs::create_dir(&path).unwrap();
        path
    }

    #[test]
    fn folder_inputs_collect_supported_images_non_recursively() {
        let directory = temp_dir();
        File::create(directory.join("a.png")).unwrap();
        File::create(directory.join("b.JPG")).unwrap();
        File::create(directory.join("notes.txt")).unwrap();
        fs::create_dir(directory.join("nested")).unwrap();
        File::create(directory.join("nested").join("c.png")).unwrap();

        let images = collect_image_paths(&[directory.display().to_string()]).unwrap();

        assert_eq!(images.len(), 2);
        assert!(images.iter().any(|path| path.ends_with("a.png")));
        assert!(images.iter().any(|path| path.ends_with("b.JPG")));

        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn glob_inputs_collect_matching_images() {
        let directory = temp_dir();
        File::create(directory.join("a.png")).unwrap();
        File::create(directory.join("b.jpg")).unwrap();

        let images = collect_image_paths(&[directory.join("*.png").display().to_string()]).unwrap();

        assert_eq!(images, vec![directory.join("a.png")]);

        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn webp_images_are_supported() {
        assert!(is_image_path(Path::new("frame.webp")));
        assert!(is_image_path(Path::new("frame.WEBP")));
    }
}
