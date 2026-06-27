use std::{
    fs::{self, File},
    io::Write,
};

use color_eyre::eyre::{Context, ContextCompat, Result};

use crate::{
    annotation::{Annotation, AnnotationFormat, LabelFileFormat, normalize_labeled_classes},
    classes::Class,
    paths::Paths,
};

#[derive(Default)]
pub struct LabelDocument {
    paths: Option<Paths>,
    dirty: bool,
    annotations: Vec<Annotation>,
    labeled_classes: Vec<Class>,
    unresolved_annotations: Vec<AnnotationFormat>,
}

impl LabelDocument {
    pub fn paths(&self) -> Option<&Paths> {
        self.paths.as_ref()
    }

    pub fn has_paths(&self, paths: &Paths) -> bool {
        self.paths
            .as_ref()
            .is_some_and(|current_paths| paths.image_path == current_paths.image_path)
    }

    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    pub fn annotations(&self) -> &[Annotation] {
        &self.annotations
    }

    pub fn annotations_mut(&mut self) -> &mut Vec<Annotation> {
        &mut self.annotations
    }

    pub fn mark_labeled_class(&mut self, class: Class) {
        if !self.labeled_classes.contains(&class) {
            self.labeled_classes.push(class);
            self.labeled_classes =
                normalize_labeled_classes(std::mem::take(&mut self.labeled_classes));
            self.dirty = true;
        }
    }

    pub fn class_is_labeled(&self, class: Class) -> bool {
        self.labeled_classes.contains(&class)
    }

    pub fn has_pending_migration_for_class(&self, class: Class) -> bool {
        self.annotations
            .iter()
            .any(|annotation| annotation.class == class && annotation.needs_point_migration())
            || self
                .unresolved_annotations
                .iter()
                .any(|annotation| annotation.class() == class && annotation.needs_point_migration())
    }

    pub fn pending_migration(&self, class: Class) -> Option<(usize, usize)> {
        let mut first_index = None;
        let mut count = 0;

        for (index, annotation) in self.annotations.iter().enumerate() {
            if annotation.class == class && annotation.needs_point_migration() {
                first_index.get_or_insert(index);
                count += 1;
            }
        }

        first_index.map(|index| (index, count))
    }

    pub fn resolve_annotations(&mut self, image_size: [f32; 2]) {
        self.annotations = self
            .unresolved_annotations
            .drain(..)
            .map(|annotation| Annotation::from_format(annotation, image_size))
            .collect();
    }

    pub fn load_new_image_with_labels(
        &mut self,
        paths: Paths,
        model_annotations: &[AnnotationFormat],
    ) -> Result<()> {
        let label_file = if paths.label_path.exists() {
            let existing_annotations =
                fs::read_to_string(&paths.label_path).wrap_err_with(|| {
                    format!("failed to read label file {}", paths.label_path.display())
                })?;
            serde_json::from_str(&existing_annotations).wrap_err_with(|| {
                format!("failed to parse label file {}", paths.label_path.display())
            })?
        } else {
            LabelFileFormat::from_unlabeled_annotations(model_annotations.to_vec())
        };

        self.annotations.clear();
        self.labeled_classes = label_file.labeled_classes;
        self.unresolved_annotations = label_file.annotations;
        self.dirty = false;
        self.paths = Some(paths);

        Ok(())
    }

    pub fn save(&mut self, image_size: Option<[f32; 2]>) -> Result<()> {
        let paths = self.paths.as_mut().wrap_err("no image loaded currently")?;
        let annotations = if let Some(image_size) = image_size {
            self.annotations
                .iter()
                .map(|annotation| annotation.to_format(image_size))
                .collect()
        } else {
            self.unresolved_annotations.clone()
        };
        let label_file = LabelFileFormat {
            labeled_classes: normalize_labeled_classes(self.labeled_classes.clone()),
            annotations,
        };

        let annotations = serde_json::to_string_pretty(&label_file).wrap_err_with(|| {
            format!(
                "failed to serialize labels for {}",
                paths.label_path.display()
            )
        })?;

        let mut file = File::create(&paths.label_path).wrap_err_with(|| {
            format!("failed to create label file {}", paths.label_path.display())
        })?;
        file.write_all(annotations.as_bytes()).wrap_err_with(|| {
            format!("failed to write label file {}", paths.label_path.display())
        })?;

        paths.check_existence();
        self.dirty = false;

        Ok(())
    }
}
