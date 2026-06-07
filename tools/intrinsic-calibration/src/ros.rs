use std::sync::Arc;

use color_eyre::Result;
use ros_z::prelude::*;
use tokio::{runtime::Runtime, sync::watch, task::JoinHandle};
use types::{stereo_image_pair::StereoImagePair, time_wrapper::TimeWrapper};

use crate::args::{Args, derive_namespace};

pub(crate) struct RosState {
    stereo_receiver: watch::Receiver<Option<Arc<TimeWrapper<StereoImagePair>>>>,
    pub(crate) namespace: String,
    pub(crate) stereo_topic: String,
    _context: Arc<Context>,
    _runtime: Runtime,
    _stereo_task: JoinHandle<()>,
}

pub(crate) struct RosResources {
    stereo_receiver: watch::Receiver<Option<Arc<TimeWrapper<StereoImagePair>>>>,
    namespace: String,
    stereo_topic: String,
    context: Arc<Context>,
    stereo_task: JoinHandle<()>,
}

impl RosState {
    pub(crate) fn new(resources: RosResources, runtime: Runtime) -> Self {
        Self {
            stereo_receiver: resources.stereo_receiver,
            namespace: resources.namespace,
            stereo_topic: resources.stereo_topic,
            _context: resources.context,
            _runtime: runtime,
            _stereo_task: resources.stereo_task,
        }
    }

    pub(crate) fn latest_stereo_pair(&self) -> Option<Arc<TimeWrapper<StereoImagePair>>> {
        self.stereo_receiver.borrow().clone()
    }
}

pub(crate) async fn create_ros_resources(args: Args) -> Result<RosResources> {
    let namespace = derive_namespace(&args.robot);
    let mut builder = ContextBuilder::default().with_namespace(&namespace);
    if let Some(router) = args.router {
        builder = builder.with_mode("client").with_router_endpoint(router)?;
    }

    let context = Arc::new(builder.build().await?);
    let node = context.create_node("intrinsic_calibration").build().await?;
    let subscriber = node
        .subscriber::<TimeWrapper<StereoImagePair>>(&args.stereo_topic)?
        .build()
        .await?;
    let (sender, stereo_receiver) = watch::channel(None);
    let stereo_task = tokio::spawn(async move {
        while let Ok(pair) = subscriber.recv().await {
            let _ = sender.send(Some(Arc::new(pair)));
        }
    });

    Ok(RosResources {
        stereo_receiver,
        namespace,
        stereo_topic: args.stereo_topic,
        context,
        stereo_task,
    })
}
