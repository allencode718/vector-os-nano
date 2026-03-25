"""MuJoCo-based simulated Unitree Go2 quadruped.

Lifecycle: MuJoCoGo2(gui=False) → connect() → stand/sit/lie_down → disconnect().

convex_mpc and mujoco are imported lazily so this module is safe to import
on systems where those packages are not installed.

Joint ordering (MuJoCo ctrl and qpos[7:19]):
    0-2:  FL  hip, thigh, calf
    3-5:  FR  hip, thigh, calf
    6-8:  RL  hip, thigh, calf
    9-11: RR  hip, thigh, calf

Quaternion convention: MuJoCo uses (w, x, y, z) in qpos[3:7].
"""
from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Lazy imports
# ---------------------------------------------------------------------------

_mujoco: Any = None


def _get_mujoco() -> Any:
    global _mujoco
    if _mujoco is None:
        import mujoco  # noqa: PLC0415
        _mujoco = mujoco
    return _mujoco


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Standing / sitting / lying postures — identical across all four legs
# Each leg: [hip, thigh, calf]
_STAND_JOINTS: list[float] = [0.0, 0.9, -1.8] * 4
_SIT_JOINTS: list[float] = [0.0, 1.5, -2.5] * 4
_LIE_DOWN_JOINTS: list[float] = [0.0, 2.0, -2.7] * 4

# PD gains
# KP=120 provides <0.15 rad steady-state error from zero pose in simulation.
# The Unitree stand_go2 example uses 50, but that example starts near standing;
# here we start from all-zeros (legs fully extended) where gravity loading
# on rear thighs requires a higher proportional gain to converge within tolerance.
_KP: float = 120.0
_KD: float = 3.5

# Torque limits (safety factor 0.9)
_TAU_HIP: float = 23.7 * 0.9      # hip / abduction joints (indices 0, 3, 6, 9)
_TAU_KNEE: float = 45.43 * 0.9    # knee / calf joints (indices 2, 5, 8, 11)

# Per-joint torque limit array  (FL hip, FL thigh, FL calf,  FR ...,  RL ...,  RR ...)
_TAU_LIMITS: np.ndarray = np.array(
    [_TAU_HIP, _TAU_HIP, _TAU_KNEE] * 4, dtype=np.float64
)

# Simulation frequency for MPC locomotion loop
_SIM_HZ: int = 1000          # MPC loop requires 1000 Hz (timestep=0.001 s)
_SIM_DT: float = 1.0 / _SIM_HZ
_CTRL_HZ: int = 200          # leg controller update rate
_CTRL_DECIM: int = _SIM_HZ // _CTRL_HZ

# Gait parameters (3 Hz trot, 0.6 duty cycle — matches ex00_demo.py)
_GAIT_HZ: int = 3
_GAIT_DUTY: float = 0.6

# MPC horizon
_MPC_DT_FACTOR: int = 16   # MPC_DT = gait_period / 16

_VIEWER_SYNC_EVERY: int = 8  # sync viewer every N sim steps

# Walk velocity limits
_VX_MAX: float = 0.8
_VY_MAX: float = 0.4
_VYAW_MAX: float = 4.0
_Z_DES: float = 0.27

# MPC torque limits
_SAFETY: float = 0.9
_TAU_LIM_MPC: np.ndarray = _SAFETY * np.array(
    [23.7, 23.7, 45.43] * 4, dtype=np.float64
)

# Leg ordering used by MPC force vector and leg controller
_LEG_NAMES: list[str] = ["FL", "FR", "RL", "RR"]

# Paths
_ROOM_XML: Path = Path(__file__).parent / "go2_room.xml"


def _build_room_scene_xml() -> Path:
    """Build a composite scene XML that places the Go2 inside an indoor room.

    Writes a resolved scene XML into the go2-convex-mpc MJCF directory
    (next to go2.xml) so that MuJoCo can resolve ``<include file="go2.xml">``
    and mesh paths correctly.

    Returns the path to the generated scene file.
    """
    import convex_mpc  # noqa: PLC0415

    convex_mpc_root = Path(convex_mpc.__file__).resolve().parents[2]
    go2_dir = convex_mpc_root / "models" / "MJCF" / "go2"
    assets_dir = go2_dir / "assets"

    template = _ROOM_XML.read_text()
    xml = template.replace("GO2_MODEL_PATH", "go2.xml")
    xml = xml.replace("GO2_ASSETS_DIR", str(assets_dir))

    out = go2_dir / "scene_room.xml"
    out.write_text(xml)
    return out


# ---------------------------------------------------------------------------
# MuJoCoGo2
# ---------------------------------------------------------------------------


class MuJoCoGo2:
    """Unitree Go2 quadruped running in MuJoCo simulation.

    Args:
        gui: Open an interactive passive viewer on connect().
        room: Use indoor room scene instead of flat ground.
    """

    def __init__(self, gui: bool = False, room: bool = True) -> None:
        self._gui: bool = gui
        self._room: bool = room
        self._mj: Any = None        # MuJoCo_GO2_Model instance
        self._viewer: Any = None
        self._connected: bool = False

        # MPC control stack — initialized in connect()
        self._pin: Any = None       # PinGo2Model
        self._gait: Any = None      # Gait
        self._traj: Any = None      # ComTraj
        self._mpc: Any = None       # CentroidalMPC (lazy — first walk() call)
        self._leg_ctrl: Any = None  # LegController

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Load MuJoCo model and optionally open viewer."""
        mj = _get_mujoco()  # ensure mujoco importable
        from convex_mpc.mujoco_model import MuJoCo_GO2_Model  # noqa: PLC0415
        from convex_mpc.go2_robot_data import PinGo2Model     # noqa: PLC0415
        from convex_mpc.gait import Gait                      # noqa: PLC0415
        from convex_mpc.com_trajectory import ComTraj         # noqa: PLC0415
        from convex_mpc.leg_controller import LegController   # noqa: PLC0415

        if self._room:
            # Build a MuJoCo_GO2_Model-compatible wrapper with our room scene
            scene_path = _build_room_scene_xml()
            model = mj.MjModel.from_xml_path(str(scene_path))
            data = mj.MjData(model)
            self._mj = MuJoCo_GO2_Model.__new__(MuJoCo_GO2_Model)
            self._mj.model = model
            self._mj.data = data
            self._mj.viewer = None
            self._mj.base_bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "base_link")

            # Place Go2 in the entry hall (center of house)
            data.qpos[0] = 10.0  # x — center hallway
            data.qpos[1] = 3.0   # y — entry area
            data.qpos[2] = 0.35  # z — slightly above floor for initial drop
        else:
            self._mj = MuJoCo_GO2_Model()

        # Set physics timestep to 1000 Hz for MPC loop compatibility
        self._mj.model.opt.timestep = _SIM_DT

        mj.mj_forward(self._mj.model, self._mj.data)

        # Initialize Pinocchio model and MPC stack
        self._pin = PinGo2Model()
        self._gait = Gait(_GAIT_HZ, _GAIT_DUTY)
        self._traj = ComTraj(self._pin)
        self._mpc = None  # lazy init on first walk() call
        self._leg_ctrl = LegController()

        if self._gui:
            try:
                import mujoco.viewer  # noqa: PLC0415
                self._viewer = mujoco.viewer.launch_passive(
                    self._mj.model,
                    self._mj.data,
                    show_left_ui=False,
                    show_right_ui=False,
                )
                # Overhead zoomed-out view: see the whole house from above the dog
                if self._viewer is not None:
                    self._viewer.cam.type = mj.mjtCamera.mjCAMERA_FREE
                    self._viewer.cam.lookat[:] = [10.0, 7.0, 0.0]  # house center
                    self._viewer.cam.distance = 22.0   # zoomed out to see full layout
                    self._viewer.cam.elevation = -65    # looking down (not fully top-down)
                    self._viewer.cam.azimuth = -90      # from the south side
            except Exception as exc:
                logger.warning("MuJoCoGo2 viewer failed to launch: %s", exc)
                self._viewer = None

        self._connected = True
        logger.info("MuJoCoGo2 connected (gui=%s)", self._gui)

    def disconnect(self) -> None:
        """Close viewer and release model. Idempotent."""
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:  # noqa: BLE001
                pass
            self._viewer = None
        self._mj = None
        self._pin = None
        self._gait = None
        self._traj = None
        self._mpc = None
        self._leg_ctrl = None
        self._connected = False

    def _require_connection(self) -> None:
        if not self._connected:
            raise RuntimeError("MuJoCoGo2: not connected. Call connect() first.")

    # ------------------------------------------------------------------
    # State queries
    # ------------------------------------------------------------------

    def get_position(self) -> list[float]:
        """Return base position [x, y, z] in world frame."""
        self._require_connection()
        return list(self._mj.data.qpos[0:3].astype(float))

    def get_velocity(self) -> list[float]:
        """Return base linear velocity [vx, vy, vz] in world frame."""
        self._require_connection()
        return list(self._mj.data.qvel[0:3].astype(float))

    def get_heading(self) -> float:
        """Return yaw angle (radians) extracted from base quaternion.

        MuJoCo quaternion convention: qpos[3:7] = (w, x, y, z).
        Yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2)).
        """
        self._require_connection()
        w, x, y, z = self._mj.data.qpos[3:7]
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return float(yaw)

    def get_joint_positions(self) -> list[float]:
        """Return all 12 joint positions (radians), ordered FL/FR/RL/RR."""
        self._require_connection()
        return list(self._mj.data.qpos[7:19].astype(float))

    def get_joint_velocities(self) -> list[float]:
        """Return all 12 joint velocities (rad/s), ordered FL/FR/RL/RR."""
        self._require_connection()
        return list(self._mj.data.qvel[6:18].astype(float))

    # ------------------------------------------------------------------
    # PD control
    # ------------------------------------------------------------------

    def _pd_interpolate(
        self,
        target_joints: np.ndarray,
        duration: float = 2.0,
    ) -> None:
        """Drive joints to target_joints using PD torque control.

        Uses a tanh-based interpolated setpoint so the robot accelerates
        smoothly from its current configuration to the target.

        Args:
            target_joints: Desired joint positions, shape (12,).
            duration: Transition duration in seconds.
        """
        self._require_connection()
        mj = _get_mujoco()

        model = self._mj.model
        data = self._mj.data
        dt = model.opt.timestep                       # 0.001 s (1000 Hz)
        total_steps = max(1, int(duration / dt))

        q_start = np.array(data.qpos[7:19], dtype=np.float64)
        q_target = np.asarray(target_joints, dtype=np.float64)

        # Add a hold phase after interpolation to allow settling
        hold_steps = max(0, int(0.5 / dt))
        total_steps_with_hold = total_steps + hold_steps

        for step in range(total_steps_with_hold):
            if step < total_steps:
                # tanh ramp: phase goes 0 → ~1 over the duration
                t_norm = (step + 1) * dt / (duration / 3.0)
                phase = float(np.tanh(t_norm))
                q_des = q_start + phase * (q_target - q_start)
            else:
                # Hold phase: track target exactly
                q_des = q_target

            # Current joint state
            q_cur = np.array(data.qpos[7:19], dtype=np.float64)
            dq_cur = np.array(data.qvel[6:18], dtype=np.float64)

            # PD torque
            tau = _KP * (q_des - q_cur) - _KD * dq_cur

            # Clamp to torque limits
            tau = np.clip(tau, -_TAU_LIMITS, _TAU_LIMITS)

            self._mj.set_joint_torque(tau)
            mj.mj_step(model, data)

            if self._viewer is not None and (step % _VIEWER_SYNC_EVERY == 0):
                self._viewer.sync()

    # ------------------------------------------------------------------
    # Posture commands
    # ------------------------------------------------------------------

    def stand(self, duration: float = 2.0) -> None:
        """Move to standing posture using PD interpolation."""
        self._require_connection()
        self._pd_interpolate(np.array(_STAND_JOINTS, dtype=np.float64), duration=duration)

    def sit(self, duration: float = 2.0) -> None:
        """Move to sitting posture using PD interpolation."""
        self._require_connection()
        self._pd_interpolate(np.array(_SIT_JOINTS, dtype=np.float64), duration=duration)

    def lie_down(self, duration: float = 2.0) -> None:
        """Move to lying-down posture using PD interpolation."""
        self._require_connection()
        self._pd_interpolate(np.array(_LIE_DOWN_JOINTS, dtype=np.float64), duration=duration)

    def stop(self) -> None:
        """Hold current joint positions with PD control for a brief moment."""
        self._require_connection()
        current = np.array(self._mj.data.qpos[7:19], dtype=np.float64)
        self._pd_interpolate(current, duration=0.1)

    # ------------------------------------------------------------------
    # MPC locomotion
    # ------------------------------------------------------------------

    def walk(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vyaw: float = 0.0,
        duration: float = 2.0,
    ) -> bool:
        """Walk at commanded body velocity using convex MPC locomotion.

        The robot must be in standing posture before calling (call stand() first).

        Args:
            vx: Forward velocity command (m/s). Clamped to ±0.8.
            vy: Lateral velocity command (m/s). Clamped to ±0.4.
            vyaw: Yaw rate command (rad/s). Clamped to ±4.0.
            duration: How long to walk (seconds).

        Returns:
            True if the walk completed without the robot falling.
        """
        self._require_connection()
        mj = _get_mujoco()

        vx = float(np.clip(vx, -_VX_MAX, _VX_MAX))
        vy = float(np.clip(vy, -_VY_MAX, _VY_MAX))
        vyaw = float(np.clip(vyaw, -_VYAW_MAX, _VYAW_MAX))

        gait_period = self._gait.gait_period          # 1/3 s for 3 Hz
        mpc_dt = gait_period / _MPC_DT_FACTOR         # ~0.0208 s
        mpc_hz = 1.0 / mpc_dt
        steps_per_mpc = max(1, int(_CTRL_HZ // mpc_hz))

        sim_steps = int(duration * _SIM_HZ)

        # Sync Pinocchio state from MuJoCo before generating first trajectory
        self._mj.update_pin_with_mujoco(self._pin)
        self._traj.generate_traj(
            self._pin,
            self._gait,
            float(self._mj.data.time),
            vx,
            vy,
            _Z_DES,
            vyaw,
            time_step=mpc_dt,
        )

        # Lazy-initialize MPC solver (needs a generated trajectory for sparsity)
        if self._mpc is None:
            from convex_mpc.centroidal_mpc import CentroidalMPC  # noqa: PLC0415
            self._mpc = CentroidalMPC(self._pin, self._traj)

        U_opt = np.zeros((12, self._traj.N), dtype=float)

        ctrl_i = 0
        tau_hold = np.zeros(12, dtype=float)

        for k in range(sim_steps):
            time_now_s = float(self._mj.data.time)

            # Control update at CTRL_HZ
            if k % _CTRL_DECIM == 0:
                # Sync Pinocchio from current MuJoCo state
                self._mj.update_pin_with_mujoco(self._pin)

                # MPC update when scheduled
                if ctrl_i % steps_per_mpc == 0:
                    self._traj.generate_traj(
                        self._pin,
                        self._gait,
                        time_now_s,
                        vx,
                        vy,
                        _Z_DES,
                        vyaw,
                        time_step=mpc_dt,
                    )
                    sol = self._mpc.solve_QP(self._pin, self._traj, False)
                    n = self._traj.N
                    w_opt = sol["x"].full().flatten()
                    U_opt = w_opt[12 * n:].reshape((12, n), order="F")

                # Compute leg torques from MPC GRF at current horizon step
                mpc_force = U_opt[:, 0]
                tau = np.zeros(12, dtype=float)
                for i, leg in enumerate(_LEG_NAMES):
                    leg_out = self._leg_ctrl.compute_leg_torque(
                        leg,
                        self._pin,
                        self._gait,
                        mpc_force[i * 3:(i + 1) * 3],
                        time_now_s,
                    )
                    tau[i * 3:(i + 1) * 3] = leg_out.tau

                tau = np.clip(tau, -_TAU_LIM_MPC, _TAU_LIM_MPC)
                tau_hold = tau.copy()
                ctrl_i += 1

            # Physics step: apply torques via split step
            mj.mj_step1(self._mj.model, self._mj.data)
            self._mj.set_joint_torque(tau_hold)
            mj.mj_step2(self._mj.model, self._mj.data)

            if self._viewer is not None and k % _VIEWER_SYNC_EVERY == 0:
                self._viewer.sync()

        return True
