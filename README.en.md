# lyf asks for help again

[中文](README.md) · [English](README.en.md)

## The gimbal moves. What comes next?

In the [control assignment](https://github.com/HKUAstar/control-training-template),
lyf handed the yaw/pitch gimbal teaching rig to the control group. For this scenario,
assume they have finished the firmware: the rig can home, follow target angles,
and stop its output during debugging.

Watching the gimbal finally hold still, lyf asks: “Can we make it turn toward a target
by itself now?”

The control group wants to try that too. They have an interface for target angles and
want to connect a vision stack. The vision group still needs to answer two questions:
which part of the camera image is an armor plate, and where will that plate be by the
time a projectile reaches it?

Your job is to build one part of that vision pipeline. We provide recordings, a model,
calibration, and an aiming simulator so you can work on your own computer. You do not
need the physical rig, camera, motors, or control board. Communication with the control
firmware is outside this assignment; the integration story supplies the context.

## Choose one path

**Complete either detection or aiming. You are not required to do both, and neither
path depends on completing the other.**

| Path | Your work | Provided | Start here |
| --- | --- | --- | --- |
| Detection | Detect armor in video, recover 3D corners with PnP, export JSONL | Pretrained model, competition clips, camera calibration | [detection/README.en.md](detection/README.en.md) |
| Aiming | Predict armor motion from delayed target states, choose aim angles and when to fire | Perfect historical observations, trajectory helper, 3D simulator, MCAP replay | [aiming/README.en.md](aiming/README.en.md) |

Detection accepts any language; Python, C++, and Rust are recommended. No model training
is needed. Aiming uses Rust and requires no detector, tracker, or state estimator.
References and AI tools are allowed for either path, but you should be able to explain
your implementation and debug its behavior.

## Start and submit

1. Open the [assignment template](https://github.com/HKUAstar/vision-training-assignment),
   select **Use this template → Create a new repository**, and create a repository
   under your account.
2. Clone your repository, choose detection or aiming, and follow that path's README.
3. Commit and push your completed code to your repository, then submit its **GitHub URL**.
