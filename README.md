````md
# Hiwonder JetRover (Standard Wheels) — ROS2 On-Robot Guide

This repository is a **course-style ROS2 guide** for working with the **Hiwonder JetRover robot (standard wheels)**.

All ROS2 code is executed **directly on the robot**, while the laptop connects via **NoMachine** over the same Wi-Fi network.

---

## What this repository covers
- ROS2 workspace setup on the JetRover
- Camera bring-up and topic inspection
- Robot movement using ROS2 velocity commands
- Creating custom ROS2 Python packages
- Foundation for vision-based and AI-driven robotics projects

---

## Setup assumptions
- Robot and laptop are connected to the **same Wi-Fi**
- You connect to the robot using **NoMachine**
- ROS2 runs **on the robot**, not on the laptop

---

## Quick start (run on the robot)

```bash
mkdir -p ~/jetrover_ws/src
cd ~/jetrover_ws
colcon build
source install/setup.bash
````

Check ROS2:

```bash
ros2 topic list
```

---

## Learning path

Follow these files in order:

1. `docs/01-setup.md`
2. `docs/02-ros2-basics.md`
3. `docs/03-camera.md`
4. `docs/04-movement.md`
5. `docs/05-build-your-own-package.md`
6. `docs/troubleshooting.md`

---

## Notes

This repository is intentionally beginner-friendly and hardware-focused.
It serves as the base for advanced projects such as:

* Vision-based navigation
* AI-controlled robots
* Multi-robot coordination and choreography

---

## License

This project was developed as part of academic coursework at Macquarie University.
Reuse or redistribution should comply with university academic policies.

```
