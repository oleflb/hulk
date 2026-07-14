use std::sync::Arc;

use color_eyre::Result;
use ros_z::{node::Node, prelude::*};
use ros_z_debug::{CachedSubscription, CachedSubscriptionNodeExt, CachedSubscriptionStatus};
use tokio::runtime::Runtime;
use types::stereo_image_pair::StereoImagePair;

use crate::args::{Args, derive_namespace};

pub(crate) struct RosState {
    stereo: CachedSubscription<StereoImagePair>,
    pub(crate) namespace: String,
    pub(crate) stereo_topic: String,
    _context: Arc<Context>,
    _node: Arc<Node>,
    _runtime: Runtime,
}

pub(crate) struct RosResources {
    stereo: CachedSubscription<StereoImagePair>,
    namespace: String,
    stereo_topic: String,
    context: Arc<Context>,
    node: Arc<Node>,
}

impl RosState {
    pub(crate) fn new(resources: RosResources, runtime: Runtime) -> Self {
        Self {
            stereo: resources.stereo,
            namespace: resources.namespace,
            stereo_topic: resources.stereo_topic,
            _context: resources.context,
            _node: resources.node,
            _runtime: runtime,
        }
    }

    pub(crate) fn latest_stereo_pair(
        &self,
    ) -> Option<Arc<ros_z_debug::SampleRecord<StereoImagePair>>> {
        self.stereo.latest()
    }

    pub(crate) fn stereo_subscription_error(&self) -> Option<String> {
        let snapshot = self.stereo.status();
        match snapshot.status() {
            CachedSubscriptionStatus::ProtocolError { .. }
            | CachedSubscriptionStatus::DecodeError { .. } => Some(format!(
                "stereo subscription error: {}",
                snapshot.message().unwrap_or("unknown error")
            )),
            CachedSubscriptionStatus::Closed => Some("stereo subscription closed".to_string()),
            _ => None,
        }
    }

    pub(crate) fn stereo_wait_status(&self) -> String {
        let snapshot = self.stereo.status();
        match snapshot.status() {
            CachedSubscriptionStatus::WaitingForFirstSample => {
                format!("waiting for stereo image pairs on {}", self.stereo_topic)
            }
            CachedSubscriptionStatus::Ready => "receiving stereo image pairs".to_string(),
            CachedSubscriptionStatus::ProtocolError { .. }
            | CachedSubscriptionStatus::DecodeError { .. } => self
                .stereo_subscription_error()
                .unwrap_or_else(|| "stereo subscription error".to_string()),
            CachedSubscriptionStatus::Closed => "stereo subscription closed".to_string(),
            _ => "waiting for stereo image pair".to_string(),
        }
    }
}

pub(crate) async fn create_ros_resources(args: Args) -> Result<RosResources> {
    let namespace = derive_namespace(&args.robot);
    let mut builder = ContextBuilder::default().with_namespace(&namespace);
    if let Some(router) = args.router {
        builder = builder.with_mode("client").with_router_endpoint(router)?;
    }

    let context = Arc::new(builder.build().await?);
    let node = Arc::new(
        context
            .create_node("intrinsic_calibration")
            .without_schema_service()
            .build()
            .await?,
    );
    let stereo = node
        .cached_subscription(&args.stereo_topic)?
        .target_namespace(&namespace)?
        .build_typed::<StereoImagePair>()
        .await?;
    let stereo_topic = stereo
        .status()
        .resolved_topic()
        .unwrap_or(&args.stereo_topic)
        .to_string();

    Ok(RosResources {
        stereo,
        namespace,
        stereo_topic,
        context,
        node,
    })
}
