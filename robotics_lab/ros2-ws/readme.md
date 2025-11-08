# 🐢 TurtleSim ROS 2 Bag Recording & Playback

#### heads up: before all of this build it: `colcon build`

## To Record

#### Step 1. Open a terminal and run the turtlesim node:
```bash
ros2 run turtlesim turtlesim_node
```

#### Step 2. Open a second terminal and run the teleop node:
```bash
ros2 run turtlesim turtle_teleop_key
```

#### Step 3. Open a third terminal and record the velocity topic:
```bash
ros2 bag record -o src/bag_files /turtle1/cmd_vel
```

#### Step 4. Move the turtle using arrow keys in the teleop terminal.

#### Step 5. Stop recording by pressing `Ctrl+C` in the recording terminal.

---

## To Play the Record

#### Step 1. Make sure the turtlesim node is running:
```bash
ros2 run turtlesim turtlesim_node
```

##### Step 2. From the root of your workspace, play the recorded bag:
```bash
ros2 bag play src/bag_files
```

> The turtle will replay the recorded movements.

---

## Notes

- Run all commands from the root of your workspace (e.g., `ros2-ws`).
- `src/bag_files` must contain both `metadata.yaml` and `.db3` directly.
- Do **not** manually create subfolders inside `bag_files`. Let `ros2 bag record` handle it.
