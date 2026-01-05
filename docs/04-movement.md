````md
# 04 — Robot Movement Control

This section explains how to control the **Hiwonder JetRover (standard wheels)**
using ROS2 velocity commands.

All commands are executed **on the robot**.

---

## Identify the velocity topic

List all topics:
```bash
ros2 topic list
````

Look for a topic named similar to:

```text
/cmd_vel
```

Confirm the message type:

```bash
ros2 topic type <cmd_vel_topic>
```

Expected type:

```text
geometry_msgs/msg/Twist
```

---

## Move the robot (basic test)

Publish a small forward movement:

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

---

## Stop the robot

Always test stop immediately after:

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## Keyboard teleoperation (optional)

If available in your setup:

```bash
# ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Safety notes

* Start with **low speeds** (0.05–0.1)
* Keep clear space around the robot
* Ensure stop command works before continuous movement

---

Next → `docs/05-build-your-own-package.md`

```
