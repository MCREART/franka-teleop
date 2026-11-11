"""Deterministic MuJoCo simulation, pose control, and contact-based grasp sensing."""
from __future__ import annotations

import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

# Reset AR tracking with the phone upright in the right hand, screen facing left.
# AR +x points backward, +z points left, and gravity-aligned +y points up.
AR_TO_ROBOT = np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
CONTROL_TIMEOUT = 0.5
WORKSPACE_MIN = np.array([0.20, -0.42, 0.115])
WORKSPACE_MAX = np.array([0.75, 0.42, 0.78])
HOME = np.array([0.0, -0.40, 0.0, -2.10, 0.0, 1.70, math.pi / 4])


def create_model() -> mujoco.MjModel:
    """Extend the unmodified, bundled Menagerie model with a tabletop task."""
    model_dir = Path(__file__).parent / "models" / "panda"
    root = ET.parse(model_dir / "panda.xml").getroot()
    root.find("compiler").set("meshdir", str(model_dir / "assets"))
    root.find("option").set("timestep", "0.002")
    root.find("option").set("cone", "elliptic")
    root.remove(root.find("keyframe"))
    hand = root.find(".//body[@name='hand']")
    ET.SubElement(hand, "site", name="tcp", pos="0 0 0.1034", size="0.006", rgba="0.1 0.9 0.8 0.7")
    # Give the actual finger collision surfaces enough friction to lift the cube.
    for body_name in ("left_finger", "right_finger"):
        for geom in root.findall(f".//body[@name='{body_name}']/geom"):
            if geom.get("class") != "visual":
                geom.set("friction", "1.5 0.01 0.001")
                geom.set("condim", "4")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", diffuse="0.7 0.7 0.7", ambient="0.25 0.25 0.25")
    ET.SubElement(visual, "global", azimuth="135", elevation="-25", offwidth="1280", offheight="960")
    assets = root.find("asset")
    ET.SubElement(assets, "texture", name="floor_tex", type="2d", builtin="checker", rgb1="0.14 0.17 0.21", rgb2="0.20 0.24 0.29", width="512", height="512")
    ET.SubElement(assets, "material", name="floor_mat", texture="floor_tex", texrepeat="3 3", reflectance="0.1")
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos="0.4 -0.7 1.8", dir="0 0 -1", directional="true")
    ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 0.1", material="floor_mat", friction="1 0.01 0.001")
    ET.SubElement(world, "geom", name="table", type="box", pos="0.57 0 0.05", size="0.34 0.36 0.05", rgba="0.26 0.33 0.42 1", friction="1 0.01 0.001")
    cube = ET.SubElement(world, "body", name="cube", pos="0.50 0 0.127")
    ET.SubElement(cube, "freejoint", name="cube_joint")
    ET.SubElement(cube, "geom", name="cube", type="box", size="0.025 0.025 0.025", mass="0.06", rgba="0.98 0.48 0.12 1", friction="1.5 0.01 0.001", condim="4", solref="0.006 1")
    ET.SubElement(world, "camera", name="overview", pos="1.35 -1.25 1.05", xyaxes="0.707 0.707 0 -0.35 0.35 0.866")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def _rotation_from_xyzw(values: list[float]) -> np.ndarray:
    quaternion = np.asarray(values, dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if norm < 0.5 or norm > 1.5:
        raise ValueError("quaternion must have approximately unit length")
    quaternion /= norm
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, quaternion[[3, 0, 1, 2]])
    return matrix.reshape(3, 3)


def _rotation_vector(matrix: np.ndarray) -> np.ndarray:
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, matrix.ravel())
    if quaternion[0] < 0:
        quaternion *= -1
    sine = float(np.linalg.norm(quaternion[1:]))
    if sine < 1e-9:
        return 2.0 * quaternion[1:]
    angle = 2.0 * math.atan2(sine, float(quaternion[0]))
    return quaternion[1:] * (angle / sine)


def _bounded_vector(vector: np.ndarray, limit: float) -> np.ndarray:
    return vector * min(1.0, limit / max(float(np.linalg.norm(vector)), 1e-12))


def validate_control(message: object) -> dict:
    """Validate the complete frame before any simulator state can change."""
    if not isinstance(message, dict) or message.get("type") != "control":
        raise ValueError("expected a control message")
    sequence = message.get("seq")
    if type(sequence) is not int or not 0 <= sequence <= 2**53 - 1:
        raise ValueError("seq must be a nonnegative integer")
    if type(message.get("engaged")) is not bool:
        raise ValueError("engaged must be a boolean")
    for key, count in (("position", 3), ("quaternion", 4)):
        values = message.get(key)
        if not isinstance(values, list) or len(values) != count:
            raise ValueError(f"{key} must contain {count} numbers")
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            raise ValueError(f"{key} must contain finite numbers")
    if any(abs(value) > 100.0 for value in message["position"]):
        raise ValueError("position is outside the supported AR tracking range")
    gripper = message.get("gripper")
    if type(gripper) not in (int, float) or not math.isfinite(gripper) or not 0 <= gripper <= 1:
        raise ValueError("gripper must be between zero and one")
    return {
        "seq": sequence,
        "position": np.array(message["position"], dtype=float),
        "rotation": _rotation_from_xyzw(message["quaternion"]),
        "gripper": float(gripper),
        "engaged": message["engaged"],
    }


class Simulation:
    """Own all physics state. Call from one thread, or hold the passive viewer lock."""

    def __init__(self):
        self.model = create_model()
        self.data = mujoco.MjData(self.model)
        self.ik_data = mujoco.MjData(self.model)
        self.arm_joints = np.array([self.model.joint(f"joint{i}").id for i in range(1, 8)])
        self.arm_qpos = self.model.jnt_qposadr[self.arm_joints]
        self.arm_dofs = self.model.jnt_dofadr[self.arm_joints]
        self.finger_qpos = np.array([self.model.joint(f"finger_joint{i}").qposadr[0] for i in (1, 2)])
        self.site_id = self.model.site("tcp").id
        self.cube_geom = self.model.geom("cube").id
        self.finger_bodies = [self.model.body(name).id for name in ("left_finger", "right_finger")]
        self.data.qpos[self.arm_qpos] = HOME
        self.data.qpos[self.finger_qpos] = 0.04
        self.data.ctrl[:7] = HOME
        self.data.ctrl[7] = 255
        mujoco.mj_forward(self.model, self.data)
        self.q_command = HOME.copy()
        self.target_position = self.data.site_xpos[self.site_id].copy()
        self.target_rotation = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
        self.gripper_command = 1.0
        self.tracking_active = False
        self.grasped = False
        self.grasp_force = 0.0
        self.last_control_time = None
        self.last_sequence = -1
        self._reference_position = None
        self._reference_rotation = None
        self._reference_target_position = None
        self._reference_target_rotation = None
        self._grasp_duration = 0.0
        self._release_duration = 0.0
        self._events = []
        self.message = "等待手机连接，按住遥操按钮开始"
        self._jac_position = np.zeros((3, self.model.nv))
        self._jac_rotation = np.zeros((3, self.model.nv))

    def _hold(self, message: str):
        self.tracking_active = False
        self._reference_position = None
        self.target_position = self.data.site_xpos[self.site_id].copy()
        self.target_rotation = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
        self.q_command = self.data.qpos[self.arm_qpos].copy()
        self.data.ctrl[:7] = self.q_command
        self.message = message

    def apply_control(self, message: dict, now: float | None = None):
        frame = validate_control(message)
        if frame["seq"] <= self.last_sequence:
            raise ValueError("seq must increase within the connection")
        now = time.monotonic() if now is None else now
        if self.last_control_time is not None and now - self.last_control_time > CONTROL_TIMEOUT:
            self._hold("数据超时，已保持当前位置")
        self.last_sequence = frame["seq"]
        self.last_control_time = now
        self.gripper_command = frame["gripper"]
        if not frame["engaged"]:
            if self.tracking_active:
                self._hold("遥操已松开，机械臂保持位置")
            return
        if not self.tracking_active:
            self._reference_position = frame["position"].copy()
            self._reference_rotation = frame["rotation"].copy()
            self._reference_target_position = self.data.site_xpos[self.site_id].copy()
            self._reference_target_rotation = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
            self.q_command = self.data.qpos[self.arm_qpos].copy()
        self.tracking_active = True
        self.target_position = np.clip(
            self._reference_target_position + AR_TO_ROBOT @ (frame["position"] - self._reference_position),
            WORKSPACE_MIN, WORKSPACE_MAX,
        )
        delta_rotation = frame["rotation"] @ self._reference_rotation.T
        self.target_rotation = AR_TO_ROBOT @ delta_rotation @ AR_TO_ROBOT.T @ self._reference_target_rotation
        self.message = "正在遥操"

    def disconnect(self):
        self._hold("手机已断开，机械臂保持位置")
        self.last_control_time = None
        self.last_sequence = -1

    def _advance_controller(self, dt: float):
        self.ik_data.qpos[:] = self.data.qpos
        self.ik_data.qpos[self.arm_qpos] = self.q_command
        mujoco.mj_forward(self.model, self.ik_data)
        position_error = self.target_position - self.ik_data.site_xpos[self.site_id]
        rotation = self.ik_data.site_xmat[self.site_id].reshape(3, 3)
        rotation_error = _rotation_vector(self.target_rotation @ rotation.T)
        velocity = np.r_[
            _bounded_vector(position_error * 5.0, 0.25),
            _bounded_vector(rotation_error * 4.0, 1.0),
        ]
        mujoco.mj_jacSite(self.model, self.ik_data, self._jac_position, self._jac_rotation, self.site_id)
        jacobian = np.vstack([self._jac_position[:, self.arm_dofs], self._jac_rotation[:, self.arm_dofs]])
        damping = 0.025
        inverse = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping**2 * np.eye(6), np.eye(6))
        joint_velocity = inverse @ velocity
        # Null-space attraction avoids drifting toward joint limits over repeated clutch cycles.
        joint_velocity += (np.eye(7) - inverse @ jacobian) @ (0.2 * (HOME - self.q_command))
        joint_velocity = np.clip(joint_velocity, -0.8, 0.8)
        limits = self.model.jnt_range[self.arm_joints]
        self.q_command = np.clip(self.q_command + dt * joint_velocity, limits[:, 0] + 0.03, limits[:, 1] - 0.03)
        self.q_command = np.clip(self.q_command, self.data.qpos[self.arm_qpos] - 0.20, self.data.qpos[self.arm_qpos] + 0.20)
        self.data.ctrl[:7] = self.q_command

    def _update_grasp(self, dt: float):
        forces = np.zeros(2)
        contact_force = np.zeros(6)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if self.cube_geom not in (contact.geom1, contact.geom2):
                continue
            other_geom = contact.geom2 if contact.geom1 == self.cube_geom else contact.geom1
            body = self.model.geom_bodyid[other_geom]
            if body not in self.finger_bodies:
                continue
            mujoco.mj_contactForce(self.model, self.data, index, contact_force)
            forces[self.finger_bodies.index(body)] += max(0.0, contact_force[0])
        self.grasp_force = float(min(forces))
        bilateral_contact = bool(np.all(forces > 0.3))
        if bilateral_contact:
            self._grasp_duration += dt
            self._release_duration = 0.0
        else:
            self._grasp_duration = 0.0
            self._release_duration += dt
        if not self.grasped and self._grasp_duration >= 0.06:
            self.grasped = True
            self._events.append({"type": "grasp", "grasped": True, "force": self.grasp_force})
        elif self.grasped and self._release_duration >= 0.10:
            self.grasped = False
            self._events.append({"type": "grasp", "grasped": False, "force": 0.0})

    def step(self, dt: float = 0.01, now: float | None = None):
        if not math.isfinite(dt) or dt <= 0 or dt > 0.1:
            raise ValueError("step duration must be between zero and 0.1 seconds")
        now = time.monotonic() if now is None else now
        if self.tracking_active and self.last_control_time is not None and now - self.last_control_time > CONTROL_TIMEOUT:
            self._hold("数据超时，已保持当前位置")
        substeps = max(1, round(dt / self.model.opt.timestep))
        actual_dt = substeps * self.model.opt.timestep
        if self.tracking_active:
            self._advance_controller(actual_dt)
        self.data.ctrl[7] = self.gripper_command * 255.0
        for _ in range(substeps):
            # Gravity compensation leaves position gains responsible for tracking and contact.
            self.data.qfrc_applied[self.arm_dofs] = self.data.qfrc_bias[self.arm_dofs]
            mujoco.mj_step(self.model, self.data)
            self._update_grasp(self.model.opt.timestep)
        if not np.all(np.isfinite(self.data.qpos)):
            raise RuntimeError("MuJoCo produced a non-finite state")

    def state(self) -> dict:
        return {
            "type": "state", "protocol": 1, "connected": True, "last_seq": self.last_sequence,
            "tracking_active": self.tracking_active,
            "grasped": self.grasped, "grasp_force": self.grasp_force,
            "gripper": float(np.clip(np.sum(self.data.qpos[self.finger_qpos]) / 0.08, 0, 1)),
            "gripper_command": self.gripper_command,
            "ee_position": self.data.site_xpos[self.site_id].tolist(),
            "target_position": self.target_position.tolist(),
            "joint_positions": self.data.qpos[self.arm_qpos].tolist(),
            "simulation_time": self.data.time,
            "message": self.message,
        }

    def pop_events(self) -> list[dict]:
        events, self._events = self._events, []
        return events
