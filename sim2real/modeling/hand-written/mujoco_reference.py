"""MuJoCo reference for the hand-written gimbal simulators.

This file is NOT hand-driven code. It only builds the same MuJoCo gimbal model as
sim2real/modeling/modeling.ipynb and runs it, so each hand-written notebook
can compare its own numbers against MuJoCo. The hand-written notebooks never use
MuJoCo to compute their own results.

Conventions (same as the hand simulators):
  - angles = (outer_angle, inner_angle) in rad, rates in rad/s, torques in N*m
  - the controller (if any) is called BEFORE each step with the state at the start of the step
  - the logs store the state AFTER each step, so row k is the state at t = (k + 1) * time_step
"""
import numpy as np  # type: ignore
import mujoco  # type: ignore

GRAVITY = 9.81  # m/s^2
TIME_STEP = 0.001  # s
ROD_LENGTH = 1.0  # m
ROD_MASS = 1.0  # kg
ROD_RADIUS = 0.02  # m
MOTOR_TORQUE_LIMIT = 3.0  # N*m

_inertia_across_rod_about_center = ROD_MASS * ROD_LENGTH**2 / 12
_inertia_about_rod_axis = ROD_MASS * ROD_RADIUS**2 / 2

_xml = f"""
<mujoco>
    <compiler angle="radian"/>
    <option gravity="0 0 -{GRAVITY}" timestep="{TIME_STEP}" integrator="RK4">
        <flag energy="enable"/>
    </option>
    <worldbody>
        <body name="rod" pos="0 0 0">
            <inertial pos="0 0 {ROD_LENGTH / 2}" mass="{ROD_MASS}" diaginertia="{_inertia_across_rod_about_center} {_inertia_across_rod_about_center} {_inertia_about_rod_axis}"/>
            <joint name="jx" pos="0 0 0" type="hinge" axis="1 0 0" damping="0" frictionloss="0"/>
            <joint name="jy" pos="0 0 0" type="hinge" axis="0 1 0" damping="0" frictionloss="0"/>
            <geom name="arm" type="capsule" fromto="0 0 0 0 0 {ROD_LENGTH}" size="{ROD_RADIUS}" contype="0" conaffinity="0"/>
            <site name="tip" pos="0 0 {ROD_LENGTH}" size="0.03"/>
        </body>
    </worldbody>
    <actuator>
        <motor name="motor_x" joint="jx" gear="1" ctrllimited="true" ctrlrange="-{MOTOR_TORQUE_LIMIT} {MOTOR_TORQUE_LIMIT}"/>
        <motor name="motor_y" joint="jy" gear="1" ctrllimited="true" ctrlrange="-{MOTOR_TORQUE_LIMIT} {MOTOR_TORQUE_LIMIT}"/>
    </actuator>
</mujoco>
"""

_model = mujoco.MjModel.from_xml_string(_xml)
_data = mujoco.MjData(_model)
_tip_site_id = _model.site("tip").id


def run_mujoco(
    start_angles,
    n_steps,
    viscous_damping=0.0,
    coulomb_friction=0.0,
    command_function=None,
    stop_below_horizontal=False,
    start_rates=None,
):
    """Run MuJoCo from rest (or from start_rates) and return the logs after each step.

    command_function(angles, rates, step_index) -> motor command (2,) in N*m, set before each step.
    """
    _model.dof_damping[:] = viscous_damping
    _model.dof_frictionloss[:] = coulomb_friction
    mujoco.mj_resetData(_model, _data)
    _data.qpos[:] = start_angles
    _data.qvel[:] = 0.0 if start_rates is None else start_rates
    _data.ctrl[:] = 0.0
    mujoco.mj_forward(_model, _data)

    angle_log, rate_log, energy_log, tip_height_log, command_log = [], [], [], [], []
    for step_index in range(n_steps):
        if command_function is not None:
            _data.ctrl[:] = command_function(_data.qpos.copy(), _data.qvel.copy(), step_index)
        command_log.append(_data.ctrl.copy())
        mujoco.mj_step(_model, _data)
        angle_log.append(_data.qpos.copy())
        rate_log.append(_data.qvel.copy())
        energy_log.append(_data.energy[0] + _data.energy[1])
        tip_height_log.append(_data.site_xpos[_tip_site_id][2])
        if stop_below_horizontal and tip_height_log[-1] < 0:
            break
    return {
        "angles": np.array(angle_log),
        "rates": np.array(rate_log),
        "energy": np.array(energy_log),
        "tip_height": np.array(tip_height_log),
        "command": np.array(command_log),
    }


def mujoco_mass_and_bias(angles, rates):
    """MuJoCo mass matrix (2x2, kg*m^2) and qfrc_bias (= velocity part + gravity part, N*m) at a state."""
    _data.qpos[:] = angles
    _data.qvel[:] = rates
    mujoco.mj_forward(_model, _data)
    mass_matrix = np.zeros((2, 2))
    for column_index in range(2):
        column = np.zeros(2)
        mujoco.mj_mulM(_model, _data, column, np.eye(2)[column_index])  # M times unit vector = one column
        mass_matrix[:, column_index] = column
    return mass_matrix, _data.qfrc_bias.copy()
