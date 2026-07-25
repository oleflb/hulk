use std::{ffi::CStr, ptr};

use bevy::{
    app::{App, Plugin},
    ecs::resource::Resource,
    platform::cell::SyncCell,
};
use mujoco_rs::{
    mujoco_c::{mj_recompile, mjData, mjModel, mjSpec, mjs_getError},
    prelude::{MjData, MjModel, MjSpec},
};

#[derive(Resource)]
pub struct MujocoWorld {
    spec: SyncCell<MjSpec>,
    data: Option<MjData<Box<MjModel>>>,
}

impl Default for MujocoWorld {
    fn default() -> Self {
        let mut spec = MjSpec::new();
        let model = spec.compile().expect("empty spec should compile");
        let data = MjData::new(Box::new(model));

        Self {
            spec: SyncCell::new(spec),
            data: Some(data),
        }
    }
}

impl MujocoWorld {
    pub fn spec(&mut self) -> &mut MjSpec {
        self.spec.get()
    }

    pub fn recompile(&mut self) {
        let result = unsafe {
            let data = self.data.as_mut().expect("MuJoCo data should exist");
            let model = data.model_mut().ffi_mut() as *mut mjModel;
            let data = data.ffi_mut() as *mut mjData;
            let spec = self.spec().ffi_mut() as *mut mjSpec;
            mj_recompile(spec, ptr::null(), model, data)
        };

        if result == 0 {
            return;
        }

        // MuJoCo frees model and data on compilation error.
        std::mem::forget(self.data.take().expect("MuJoCo data should exist"));

        let error_msg = unsafe {
            let ptr = mjs_getError(self.spec().ffi_mut() as *mut mjSpec);
            if ptr.is_null() {
                c"(unknown error)"
            } else {
                CStr::from_ptr(ptr)
            }
        };

        panic!("Compilation failed: {}", error_msg.to_string_lossy())
    }
}

pub struct MujocoWorldPlugin;

impl Plugin for MujocoWorldPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MujocoWorld::default());
    }
}
