use std::{collections::HashMap, fs, path::Path};

use color_eyre::{Result, eyre::Context};
use serde::{Deserialize, Serialize};

use crate::annotation::AnnotationFormat;

#[derive(Default, Serialize, Deserialize)]
pub struct ModelAnnotations {
    #[serde(flatten)]
    images: HashMap<String, Vec<AnnotationFormat>>,
}

impl ModelAnnotations {
    pub fn try_new(path: impl AsRef<Path>) -> Result<Self> {
        let file_content = fs::read_to_string(&path).wrap_err_with(|| {
            format!(
                "failed to read predictions file {}",
                path.as_ref().display()
            )
        })?;

        Ok(Self {
            images: serde_json::from_str(&file_content)
                .wrap_err_with(|| format!("failed to parse {}", path.as_ref().display()))?,
        })
    }

    pub fn for_image(&self, image_name: &str) -> Option<&[AnnotationFormat]> {
        self.images.get(image_name).map(Vec::as_slice)
    }
}
