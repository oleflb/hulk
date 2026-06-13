use clap::Parser;

#[derive(Clone, Debug, Parser)]
pub(crate) struct Arguments {
    /// Robot name used as ROS-Z namespace, e.g. robot-01. Omit for root namespace.
    #[arg(long, conflicts_with = "namespace")]
    pub(crate) robot: Option<String>,

    /// Explicit ROS-Z namespace. Omit for root namespace.
    #[arg(long)]
    pub(crate) namespace: Option<String>,

    /// Zenoh router endpoint, e.g. tcp/10.0.24.1:7447. Defaults to localhost router.
    #[arg(long)]
    pub(crate) router: Option<String>,
}

impl Arguments {
    pub(crate) fn namespace(&self) -> String {
        match (&self.namespace, &self.robot) {
            (Some(namespace), _) => normalize_namespace(namespace),
            (None, Some(robot)) => normalize_namespace(robot),
            (None, None) => "/".to_string(),
        }
    }

    pub(crate) fn router_display(&self) -> String {
        self.router
            .clone()
            .unwrap_or_else(|| "tcp/localhost:7447".to_string())
    }
}

fn normalize_namespace(namespace: &str) -> String {
    let components = namespace
        .split('/')
        .filter(|component| !component.is_empty())
        .map(sanitize_namespace_component)
        .collect::<Vec<_>>();

    if components.is_empty() {
        "/".to_string()
    } else {
        format!("/{}", components.join("/"))
    }
}

fn sanitize_namespace_component(component: &str) -> String {
    let mut sanitized = String::new();

    for character in component.chars() {
        if character.is_ascii_alphanumeric() || character == '_' {
            sanitized.push(character);
        } else {
            sanitized.push('_');
        }
    }

    if sanitized
        .chars()
        .next()
        .is_some_and(|character| character.is_ascii_digit())
    {
        sanitized.insert(0, '_');
    }

    sanitized
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robot_namespace_replaces_invalid_characters() {
        assert_eq!(normalize_namespace("robot-01"), "/robot_01");
    }

    #[test]
    fn explicit_namespace_is_normalized() {
        assert_eq!(normalize_namespace("/foo/bar-baz/"), "/foo/bar_baz");
    }
}
