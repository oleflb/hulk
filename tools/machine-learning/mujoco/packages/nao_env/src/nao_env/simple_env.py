from pathlib import Path
from typing import Any, ClassVar, override

import mujoco
import numpy as np
from gymnasium.envs.mujoco.mujoco_env import MujocoEnv
from gymnasium.spaces import Box
from nao_interface import Nao
from numpy.typing import NDArray


DEFAULT_CAMERA_CONFIG = {
    "trackbodyid": 1,
    "distance": 4.0,
    "lookat": np.array((0.0, 0.0, 0.8925)),
    "elevation": -20.0,
}


class SimpleNaoEnv(MujocoEnv):
    metadata: ClassVar[dict[str, Any]] = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
        "render_fps": 83,
    }

    def __init__(
        self,
        throw_tomatoes: bool,
        **kwargs: Any,
    ) -> None:
        observation_space = Box(
            low=-np.inf,
            high=np.inf,
            shape=(37,),
            dtype=np.float64,
        )
        MujocoEnv.__init__(
            self,
            str(Path.cwd().joinpath("model", "scene.xml")),
            frame_skip=4,
            observation_space=observation_space,
            default_camera_config=DEFAULT_CAMERA_CONFIG,
            **kwargs,
        )

    def _get_obs(self) -> NDArray[np.floating]:
        return self.data.sensordata

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        observation = self._get_obs()
        return observation, 0.0, False, False, {}

    def reset(self, *args, **kwargs):
        return self._get_obs(), {}
