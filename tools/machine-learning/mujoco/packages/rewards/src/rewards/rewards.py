import mujoco
import numpy as np
from numpy.typing import NDArray

from .base import BaseReward, RewardContext


class ConstantReward(BaseReward):
    def reward(self, context: RewardContext) -> np.floating:
        _ = context
        return np.float32(1.0)


class ControlAmplitudePenalty(BaseReward):
    def reward(self, context: RewardContext) -> np.floating:
        return np.square(context.action).sum()


class ExternalImpactForcesPenalty(BaseReward):
    def reward(self, context: RewardContext) -> np.floating:
        return np.square(context.nao.data.cfrc_ext).sum()


class ClippedHeadHeightReward(BaseReward):
    def __init__(self, clip_height: float) -> None:
        self.clip_height = clip_height

    def reward(self, context: RewardContext) -> np.floating:
        return np.square(
            np.minimum(
                context.nao.data.site("head_center").xpos[2], self.clip_height
            )
        )


class HeadZErrorPenalty(BaseReward):
    def __init__(self, target: float) -> None:
        self.target = target

    def reward(self, context: RewardContext) -> np.floating:
        return np.square(
            context.nao.data.site("head_center").xpos[2] - self.target
        )


class ActionRatePenalty(BaseReward):
    def __init__(self, control_dimension: int) -> None:
        self.last_action = np.zeros(control_dimension)
        self.is_initialized = False

    def reward(self, context: RewardContext) -> np.floating:
        action_rate = np.mean(np.square(context.action - self.last_action))
        self.last_control = np.copy(context.action)
        if not self.is_initialized:
            self.is_initialized = True
            return np.float32(0.0)
        return action_rate

    def reset(self) -> None:
        self.is_initialized = False


class FootPressureReward(BaseReward):
    def __init__(self, momentum: float = 0.99) -> None:
        self.momentum = momentum
        self.pressure = 0.0

    def reward(self, context: RewardContext) -> np.floating:
        left_pressure = np.mean(context.nao._read_left_fsr_values())
        right_pressure = np.mean(context.nao._read_right_fsr_values())
        pressure = left_pressure + right_pressure
        self.pressure = (
            self.momentum * self.pressure + (1 - self.momentum) * pressure
        )
        return self.pressure

    def reset(self) -> None:
        self.pressure = 0.0


class HeadXYErrorPenalty(BaseReward):
    def __init__(self, target: NDArray[np.floating]) -> None:
        self.target = target

    def reward(self, context: RewardContext) -> np.floating:
        head_center_xy = context.nao.data.site("head_center").xpos[:2]
        return np.mean(np.square(head_center_xy - self.target))


class HeadOverTorsoPenalty(BaseReward):
    def reward(self, context: RewardContext) -> np.floating:
        robot_xy = context.nao.data.site("Robot").xpos[:2]
        head_xy = context.nao.data.site("head_center").xpos[:2]
        return np.mean(np.square(head_xy - robot_xy))


class HeadOverFeetPenalty(BaseReward):
    def reward(self, context: RewardContext) -> np.floating:
        left_foot_xy = context.nao.data.site("left_sole").xpos[:2]
        right_foot_xy = context.nao.data.site("right_sole").xpos[:2]
        center_xy = (left_foot_xy + right_foot_xy) / 2
        head_xy = context.nao.data.site("head_center").xpos[:2]

        return np.mean(np.square(head_xy - center_xy))


class OnlyFeetHaveGroundContactPenalty(BaseReward):
    def __init__(
        self, ground: str, main_body: str, exceptions: list[str]
    ) -> None:
        self.ground = ground
        self.exceptions = exceptions
        self.main_body = main_body

        self.nao_geom_ids = None
        self.ground_geom_ids = None

    def get_sub_body_ids(
        self, model: mujoco.MjModel, body_name: str
    ) -> set[int]:
        parent_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )

        # Dictionary to store child-parent relationships
        body_children = {i: [] for i in range(model.nbody)}
        for i in range(model.nbody):
            if (
                model.body_parentid[i] >= 0
            ):  # Skip the root body (has parent ID -1)
                body_children[model.body_parentid[i]].append(i)

        def collect_descendants(body_id: int) -> set[int]:
            descendants = {body_id}
            for child_id in body_children[body_id]:
                descendants.update(collect_descendants(child_id))
            return descendants

        return collect_descendants(parent_body_id)

    def initialize(self, model: mujoco.MjModel) -> None:
        exception_ids = {
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in self.exceptions
        }

        nao_ids = self.get_sub_body_ids(model, self.main_body) - exception_ids
        self.nao_geom_ids = np.array(
            [model.body_geomadr[body_id] for body_id in nao_ids]
        )
        ground_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, self.ground
        )
        self.ground_geom_ids = np.array(model.body_geomadr[ground_id])

    def reward(self, context: RewardContext) -> np.floating:
        if self.nao_geom_ids is None or self.ground_geom_ids is None:
            self.initialize(context.nao.model)
        contacts1 = context.nao.data.contact.geom

        nao_contacts = np.isin(contacts1, self.nao_geom_ids)
        ground_contacts = np.isin(contacts1, self.ground_geom_ids)

        illegal_contacts = (nao_contacts[:, 0] & ground_contacts[:, 1]) | (
            nao_contacts[:, 1] & ground_contacts[:, 0]
        )

        return illegal_contacts.sum()


class TorqueChangeRatePenalty(BaseReward):
    def __init__(self, actuator_dimension: int, dt: float) -> None:
        self.previous_force = np.zeros(actuator_dimension)
        self.is_initialized = False
        self.dt = dt

    def reward(self, context: RewardContext) -> np.floating:
        previous_torque = (
            context.nao.model.actuator_gear[:, 0] * self.previous_force
        )
        current_torque = (
            context.nao.model.actuator_gear[:, 0]
            * context.nao.data.actuator_force
        )
        torque_change_rate = np.mean(
            np.abs(previous_torque - current_torque) / self.dt
        )
        self.previous_force = np.copy(context.nao.data.actuator_force)

        if not self.is_initialized:
            self.is_initialized = True
            return np.float32(0.0)

        return torque_change_rate

    def reset(self) -> None:
        self.is_initialized = False


class XDistanceReward(BaseReward):
    def reward(self, context: RewardContext) -> np.floating:
        return context.nao.data.site("Robot").xpos[0]


class JerkPenalty(BaseReward):
    def __init__(self, dt: float) -> None:
        self.buffer = np.zeros((4, 3))
        self._empty = np.empty((3, 3))
        self.kernel = np.array([1, -3, 3, -1]) / dt**3
        self.current_step = 0

    def reward(self, context: RewardContext) -> np.floating:
        self.current_step += 1
        self._empty = self.buffer[1:]
        self.buffer[:-1] = self._empty
        self.buffer[-1] = context.nao.data.site("Robot").xpos

        if self.current_step < 4:
            return np.float32(0.0)

        jerk = self.kernel @ self.buffer
        return np.linalg.norm(jerk)

    def reset(self) -> None:
        self.buffer[:] = 0.0
        self.current_step = 0
