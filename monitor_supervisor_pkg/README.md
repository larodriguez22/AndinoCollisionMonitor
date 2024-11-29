# Collision Monitor Supervisor
This package contains integration with nav2 collision monitor.

## Usage

### Use in sim

Run sim:

```bash
ros2 launch andino_gz andino_gz.launch.py
```

Run collision monitor:

```bash
ros2 launch nav2_collision_monitor collision_monitor_node.launch.py 
```

Run Teleop:

```bash
ros2 launch andino_bringup teleop_keyboard.launch.py 
```

Run monitor supervisor:

```bash
ros2 run monitor_supervisor_pkg monitor_supervisor_node
```

### Use with andino

Run robot:

```bash
ros2 launch andino_bringup andino_robot.launch.py
```

Run collision monitor:

```bash
ros2 launch nav2_collision_monitor collision_monitor_node.launch.py 
```

Run Teleop:

```bash
ros2 launch andino_bringup teleop_keyboard.launch.py 
```

Run monitor supervisor:

```bash
ros2 run monitor_supervisor_pkg monitor_supervisor_node
```

## Videos

### Sim
[collision_monitor.webm](https://github.com/user-attachments/assets/a2d516bf-82d9-48e7-8483-8687e97a11c2)


### Andino




