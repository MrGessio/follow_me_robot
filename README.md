# Follow Me Robot

A ROS2 node written in C++ that enables a mobile robot to autonomously detect and follow a colored object (red) using a camera and computer vision.

## Overview

The `follow_me_node` subscribes to a camera image stream, detects the target object using HSV color segmentation with OpenCV, and continuously tracks it by publishing velocity commands. The robot rotates to center the target in the frame and moves forward to maintain a safe following distance.

## How It Works

1. **Camera input** - subscribes to `/camera/image_raw` topic (`sensor_msgs/Image`)
2. **Color detection** - converts frame to HSV and applies red color thresholds
3. **Noise filtering** - morphological erosion and dilation to clean up the mask
4. **Contour detection** - finds the largest contour, assumed to be the target object
5. **Proportional control logic:**
   - Calculates horizontal error between object center and frame center
   - Rotates proportionally to center the object in the frame
   - Moves forward if the object occupies less than 5% of the frame (too far)
   - Stops when the object is close enough (area ratio ≥ 5%)
   - Stops and waits if no object is detected
6. **Velocity output** — publishes `geometry_msgs/Twist` commands to `/cmd_vel`

## Tech Stack

- **ROS2** (tested on Humble)
- **C++**
- **OpenCV** (HSV segmentation, contour detection)
- **cv_bridge** (ROS2 ↔ OpenCV image conversion)
- **image_transport**
- **Gazebo** simulation compatible

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Subscribed |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Published |

## Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `angular_gain` | `0.8` | Proportional gain for rotation |
| `linear_speed` | `0.15 m/s` | Forward speed when following |
| `target_area_ratio` | `0.05` | Target occupies ~5% of frame = desired distance |
| `min_contour_area` | `500 px²` | Minimum detection size to filter noise |

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
- OpenCV
- `cv_bridge`
- `image_transport`
- `sensor_msgs`
- `geometry_msgs`
- `rclcpp`

## Author

MrGessio - Mechatronics Engineer with hands-on experience in ROS2, industrial robots (ABB, Fanuc), and robotics simulation (Gazebo, Isaac SIM).
