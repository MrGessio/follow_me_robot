# Follow Me Robot

A ROS2 node written in C++ that enables a mobile robot to autonomously detect and follow the closest object in its environment using LiDAR sensor data.

## Overview

The `follow_me_node` subscribes to laser scan data, identifies the nearest object in the robot's surroundings and continuously tracks it by adjusting both angular and linear velocity. The robot rotates to face the target and moves forward to maintain a safe following distance.

## How It Works

1. **LiDAR input** - subscribes to `/scan` topic (`sensor_msgs/LaserScan`)
2. **Closest object detection** - scans all LiDAR readings to find the minimum valid distance and its angle
3. **Angle normalization** - converts the index to radians and normalizes to [-π, π]
4. **Reactive control logic:**
   - Object too close (< 0.3 m) → stop completely
   - Object within following range (< 0.6 m) → rotate toward it, don't advance
   - Object farther than 0.6 m → rotate toward it and move forward
   - No object detected → stop
5. **Velocity output** — publishes `geometry_msgs/Twist` commands to `/cmd_vel`

## Tech Stack

- **ROS2** (tested on Humble)
- **C++**
- **LiDAR / LaserScan** (`sensor_msgs/msg/laser_scan`)
- **TurtleBot3** compatible
- **Gazebo** simulation compatible

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscribed |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Published |

## Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `target_distance` | `0.6 m` | Desired following distance |
| `stop_distance` | `0.3 m` | Minimum safe distance - robot stops |
| `linear_speed` | `0.2 m/s` | Forward speed when following |
| `angular_speed` | `0.5 rad/s` | Rotation speed when aligning |
| `angle_threshold` | `0.15 rad` | Dead zone to prevent jitter (~8°) |

## Build & Run

```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/MrGessio/follow_me_robot.git

# Build
cd ~/ros2_ws
colcon build --packages-select follow_me_robot

# Source and run
source install/setup.bash
ros2 run follow_me_robot follow_me_node
```

## Requirements

- ROS2 Humble (or newer)
- `sensor_msgs`
- `geometry_msgs`
- `rclcpp`

## Author

Mr.Gessio - Mechatronics Engineer with hands-on experience in ROS2, industrial robots (ABB, Fanuc), and robotics simulation (Gazebo, Isaac SIM).
