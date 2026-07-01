A complete robot-motion pipeline on the **Universal Robots UR5**, implemented in MATLAB:
**forward kinematics → inverse kinematics → waypoints → quintic trajectory → all-joint PD control → tracking analysis.**

The script runs top-to-bottom, prints every FK / IK / waypoint result to the Command Window, produces all figures, and animates the arm using the **controlled** response (`qActual`).

---

## Pipeline

| Stage | What it does |
|-------|--------------|
| **Model** | Loads the UR5 (`loadrobot("universalUR5")`) — 6 revolute joints, `tool0` end-effector. |
| **Forward Kinematics** | Joint angles → end-effector pose via `getTransform`. Two configs tested. |
| **Inverse Kinematics** | Target pose → joint angles via a numerical solver, verified against FK (< 1e-6 m residual). |
| **Trajectory** | Quintic polynomial through `Home → Target 1 → Target 2 → Home`, smooth start/stop. |
| **PD Control** | Feedback controller `u = Kp·e + Kd·ė` on a decoupled joint model, run on all 6 joints. |
| **Analysis** | Desired-vs-actual comparison, tracking error, and an animation driven by `qActual`. |

---

## Results

- **FK** — Config A `[0,0,0,0,0,0]` → EE `(-0.817, -0.191, -0.005)` m; Config B `[45,-60,60,-30,90,0]°` → EE `(-0.434, -0.589, 0.416)` m
- **IK** — both targets solved, FK residual error **< 1e-6 m**
- **Control** — gains `Kp = 120`, `Kd = 22` on `J·q̈ + B·q̇ = u` (`J = 1`, `B = 2`); **RMS tracking error < 0.8° per joint**, peak ≈ 1.8° on the fastest joint (J6)

---

## How to run

1. Open `manipulator_kinematics_UR5.m` in **MATLAB R2023b+** with the **Robotics System Toolbox** installed.
2. Press **Run**. The script executes the full pipeline and generates all figures.
3. For a demo, screen-record the animation window (Figure 5).

---

## Repository contents

| File | Description |
|------|-------------|
| `manipulator_kinematics_UR5.m` | The main MATLAB script (full pipeline). |
| `fig_*.png` | Exported result figures. |
| `UR5_Reflection_Sheet_Completed.docx` | Completed reflection sheet. |
| `UR5_Kinematics_Presentation.pptx` | Presentation slides. |

---

## Limitations & next steps

The joint model is **rigid and decoupled** — no gravity, Coriolis, or friction coupling — a deliberate simplification for a controls demonstration. Natural extensions: full rigid-body dynamics with gravity, computed-torque / PID control, task-space trajectory planning, and collision / workspace checking.

---

## License

Released under the MIT License.

> Course project — MDM-M-1, Digital Tools in Development and Production (SS26).
> Numbers come from MATLAB's standard UR5 model; run the script to reproduce them (tiny differences can appear across toolbox versions).
