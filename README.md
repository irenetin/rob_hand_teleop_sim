# dexhand+arm_teleop_sim

Notes for simulation environment for **robot arm + dexterous hand teleoperation**, with a focus on **control design**.


## Control Design Overview

The control system is split into two independent parts:

- Robot arm control (end-effector pose tracking)
- Dexterous hand control (joint-level retargeting)

## Arm Control

### Method

The robot arm is controlled using an **iterative damped least-squares Jacobian inverse kinematics (IK)** method implemented in **pure Pinocchio**.

The end-effector error is defined directly on **SE(3)** using a logarithmic map. This allows translation and rotation to be corrected **together**, as a single geometric error.

At each control step:

1. Compute the relative transform between current and target end-effector pose
2. Map the relative transform to a 6D SE(3) log error
3. Solve a damped least-squares Jacobian system
4. Integrate joint updates over a fixed control timestep

This control formulation follows the **Pinocchio-based motion control style used in https://github.com/Dingry/BunnyVisionPro**.


## Why Pinocchio Instead of Pink

Earlier iterations of this project evaluated **Pink**, a QP-based differential IK framework.

While Pink provides explicit constraint handling, it was not selected for the final control stack for the following reasons:

- **Unified pose error**  
  Position and orientation are handled in a single SE(3) error, without manually tuning relative weights.

- **Lower complexity**  
  The Pinocchio-based implementation avoids external QP solvers and additional abstraction layers.

- **Better suited to teleoperation**  
  Teleoperation inputs are smooth and incremental. Iterative damped IK converges reliably in this regime.

- **Predictable real-time behavior**  
  The control loop integrates cleanly into a fixed-rate (e.g. 60 Hz) update cycle and works well with streaming inputs.


## Hand Control

Dexterous hand control is implemented via **retargeting**, mapping human hand motion to robot hand joint commands.

The retargeting module is independent from the arm IK solver and can be adapted to different hand models and joint layouts.


## Control-Related Dependencies

- Pinocchio
- Isaac Lab
- PyTorch

---

## Reference

The arm control design is inspired by the Pinocchio-based motion control approach used in:

- https://github.com/Dingry/BunnyVisionPro
