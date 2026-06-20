use std::sync::Arc;

use color_eyre::Result;
use ros_z::prelude::*;
use ros_z_debug::{SubscriptionHandle, SubscriptionManager, SubscriptionStatus};
use tokio::runtime::Runtime;
use types::{stereo_image_pair::StereoImagePair, time_wrapper::TimeWrapper};

use crate::args::{Args, derive_namespace};

pub(crate) struct RosState {
    stereo: SubscriptionHandle<TimeWrapper<StereoImagePair>>,
    pub(crate) namespace: String,
    pub(crate) stereo_topic: String,
    _context: Arc<Context>,
    _debug_manager: SubscriptionManager,
    _runtime: Runtime,
}

pub(crate) struct RosResources {
    stereo: SubscriptionHandle<TimeWrapper<StereoImagePair>>,
    namespace: String,
    stereo_topic: String,
    context: Arc<Context>,
    debug_manager: SubscriptionManager,
}

impl RosState {
    pub(crate) fn new(resources: RosResources, runtime: Runtime) -> Self {
        Self {
            stereo: resources.stereo,
            namespace: resources.namespace,
            stereo_topic: resources.stereo_topic,
            _context: resources.context,
            _debug_manager: resources.debug_manager,
            _runtime: runtime,
        }
    }

    pub(crate) fn latest_stereo_pair(
        &self,
    ) -> Option<Arc<ros_z_debug::SampleRecord<TimeWrapper<StereoImagePair>>>> {
        self.stereo.latest()
    }

    pub(crate) fn stereo_subscription_error(&self) -> Option<String> {
        let snapshot = self.stereo.status();
        match snapshot.status() {
            SubscriptionStatus::ProtocolError { .. } | SubscriptionStatus::DecodeError { .. } => {
                Some(format!(
                    "stereo subscription error: {}",
                    snapshot.message().unwrap_or("unknown error")
                ))
            }
            SubscriptionStatus::Closed => Some("stereo subscription closed".to_string()),
            _ => None,
        }
    }

    pub(crate) fn stereo_wait_status(&self) -> String {
        let publisher_count = self.stereo.publisher_count();
        let snapshot = self.stereo.status();
        match snapshot.status() {
            SubscriptionStatus::WaitingForFirstSample if publisher_count == 0 => {
                format!("waiting for publisher on {}", self.stereo_topic)
            }
            SubscriptionStatus::WaitingForFirstSample => format!(
                "waiting for stereo image pair from {} publisher(s)",
                publisher_count
            ),
            SubscriptionStatus::Ready => format!(
                "receiving stereo image pairs from {} publisher(s)",
                publisher_count
            ),
            SubscriptionStatus::ProtocolError { .. } | SubscriptionStatus::DecodeError { .. } => {
                self.stereo_subscription_error()
                    .unwrap_or_else(|| "stereo subscription error".to_string())
            }
            SubscriptionStatus::Closed => "stereo subscription closed".to_string(),
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
    let debug_manager = SubscriptionManager::new(
        node,
        ros_z_debug::ManagerOptions::with_target_namespace(&namespace)?,
    );
    let stereo = debug_manager
        .subscribe_typed::<TimeWrapper<StereoImagePair>>(&args.stereo_topic)
        .with_stamp(|message| message.time)
        .build()
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
        debug_manager,
    })
}
