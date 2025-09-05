# Demo Delayed Teleop

A ROS2 system that simulates communication delays in space robotics teleoperation, featuring dynamic delay control and safety override capabilities.

## Overview

This package demonstrates realistic space teleoperation scenarios where communication delays between Earth and celestial bodies affect robot control. The system provides:

- **Dynamic delay simulation** (Earth: 0s, Moon: 1.25s, Artificial: 2.5s)
- **Safety override system** for emergency control
- **Multi-modal delay** affecting both commands and camera feeds
- **Real-time delay switching** during operation

## Architecture

```
Joy User ──────┐
               ├──► Safety Node ──► cmd_vel_user ──► Delay Node ──► cmd_vel_out ──► Fleet Robot
Joy Safety ────┘                                                      ▲
                                                                       │
Camera Raw ────────────────────────────────────────────────────────────┘
```

## Components

### 1. Delay Node (`delay_node.cpp`)
- **Purpose**: Applies configurable delays to incoming messages
- **Input Topics**: 
  - `/cmd_vel_user` (geometry_msgs/TwistStamped)
  - `/camera/image_raw` (sensor_msgs/Image)
- **Output Topics**: 
  - `/cmd_vel_out` (geometry_msgs/TwistStamped)
  - `/camera/image_delayed` (sensor_msgs/Image)
- **Parameters**: `delay_sec` (double) - Delay in seconds

### 2. Safety Node (`safety_node.py`)
- **Purpose**: Manages user/safety control arbitration and delay modes
- **Input Topics**:
  - `/joy_user` (sensor_msgs/Joy) - User joystick commands
  - `/joy_safety` (sensor_msgs/Joy) - Safety operator commands
  - `/cmd_vel_out` (geometry_msgs/Twist) - Delayed commands from delay_node
- **Output Topics**:
  - `/cmd_vel_user` (geometry_msgs/Twist) - User commands to delay system
  - `/fleet_0/cmd_vel` (geometry_msgs/Twist) - Direct robot commands

## Installation

### Prerequisites
- ROS2 Jazzy
- C++17 compiler
- Python 3.8+

### Build Instructions

```bash
# Clone the repository
cd ~/ros_ws/src
git clone <your-repo-url> demo_delayed_teleop

# Install dependencies
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select demo_delayed_teleop

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the Complete System

```bash
ros2 launch demo_delayed_teleop teleop.launch.py
```

### Manual Component Launch

```bash
# Terminal 1: Delay Node
ros2 run demo_delayed_teleop delay_node

# Terminal 2: Safety Node
ros2 run demo_delayed_teleop safety_node.py
```

### Control Commands

#### Safety Joystick Commands
- **Button 0**: Activate safety override (emergency control)
- **Button 1**: Return control to user
- **Button 2**: Moon mode (1.25s delay)
- **Button 3**: Earth mode (no delay)
- **Button 4**: Artificial delay (2.5s)

#### Testing Without Physical Joystick

```bash
# Set Moon mode (1.25s delay)
ros2 topic pub --once /joy_safety sensor_msgs/msg/Joy '{
  axes: [0.0, 0.0, 0.0, 0.0], 
  buttons: [0, 0, 1, 0, 0, 0, 0, 0]
}'

# Send user movement command
ros2 topic pub /joy_user sensor_msgs/msg/Joy '{
  axes: [0.0, 1.0, 0.0, 0.0], 
  buttons: [0, 0, 0, 0, 0, 0, 0, 0]
}' --rate 2

# Activate safety override with movement
ros2 topic pub --once /joy_safety sensor_msgs/msg/Joy '{
  axes: [0.0, 0.5, 0.0, 0.0], 
  buttons: [1, 0, 0, 0, 0, 0, 0, 0]
}'

# Return to Earth mode (no delay)
ros2 topic pub --once /joy_safety sensor_msgs/msg/Joy '{
  axes: [0.0, 0.0, 0.0, 0.0], 
  buttons: [0, 0, 0, 1, 0, 0, 0, 0]
}'
```

### Dynamic Delay Control

```bash
# Change delay during runtime
ros2 param set /delay_node delay_sec 3.0

# Check current delay
ros2 param get /delay_node delay_sec
```

### Monitoring

```bash
# Monitor delayed commands
ros2 topic echo /cmd_vel_out

# Monitor direct robot commands
ros2 topic echo /fleet_0/cmd_vel

# Check message rates
ros2 topic hz /cmd_vel_user
ros2 topic hz /cmd_vel_out

# View system graph
rqt_graph
```

## Key Features

### 🚀 Realistic Space Communication Delays
- **Earth**: Immediate control (0s delay)
- **Moon**: 1.25s average delay (realistic Earth-Moon communication)
- **Custom**: Configurable delays for other scenarios

### 🛡️ Safety Override System
- Emergency takeover capability
- Immediate direct control bypassing delays
- Seamless switching between user and safety control

### ⚡ Real-time Performance
- Thread-based delay implementation
- Non-blocking message processing
- Individual message timing (not queue-based)

### 🔧 Dynamic Configuration
- Runtime delay adjustment
- Parameter-based control
- Hot-swappable delay modes

## Technical Implementation

### Delay Implementation
The delay system uses `std::thread` with `std::this_thread::sleep_for()` to create individual delays for each message, similar to the proven `boost::asio::deadline_timer` approach from ROS1.

```cpp
std::thread([this, msg, delay = current_delay_]() {
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));
    if (rclcpp::ok() && pub_) {
        pub_->publish(*msg);
    }
}).detach();
```

### Safety Architecture
The safety node acts as a message router and arbitrator:
- **Normal Operation**: User → Safety Node → Delay Node → Robot
- **Emergency Mode**: Safety → Direct → Robot (bypassing delays)

## Troubleshooting

### Common Issues

1. **No delayed output**: Check if delay_node is receiving messages:
   ```bash
   ros2 topic echo /cmd_vel_user
   ```

2. **Joy index errors**: Ensure Joy messages have sufficient axes:
   ```bash
   # Correct format with 4 axes
   ros2 topic pub /joy_user sensor_msgs/msg/Joy '{axes: [0.0, 1.0, 0.0, 0.0]}'
   ```

3. **Parameter setting fails**: Verify delay_node is running:
   ```bash
   ros2 node list | grep delay_node
   ```

### Debug Mode
Enable detailed logging:
```bash
ros2 run demo_delayed_teleop delay_node --ros-args --log-level debug
```

## Integration with Gazebo

For integration with Gazebo simulation:

```bash
# Bridge if needed
ros2 run ros_gz_bridge parameter_bridge /fleet_0/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# Remap topics in launch file for your robot namespace
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Inspired by real space mission teleoperation challenges
- Built with ROS2 Jazzy and modern C++17 features
- Designed for space robotics research and education