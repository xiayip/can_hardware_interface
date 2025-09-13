# CAN Hardware Interface for Piper Robot Arm

This repository contains the ROS 2 hardware interface implementation for controlling the Piper robot arm via CAN bus communication. It provides a complete solution for integrating the Piper arm with ROS 2 control frameworks, including MoveIt2 and servo control.

## Repository Structure

```
can_hardware_interface/
├── can_data_plugins/           # CAN data processing plugins
│   ├── include/can_data_plugins/
│   │   └── piper_feedback/     # Piper-specific feedback handlers
│   └── src/
│       ├── omni_picker/        # Omni picker control
│       ├── piper_ctrl/         # Piper control plugins  
│       ├── piper_feedback/     # Piper feedback plugins
│       └── piper_mit_ctrl/     # MIT control mode plugins
├── piper_bringup/              # Launch files and robot configuration
│   ├── config/
│   ├── launch/
│   │   └── piper_bringup.launch.py  # Main launch file
│   ├── meshes/                 # 3D mesh files for visualization
│   ├── rviz/                   # RViz configuration files
│   └── urdf/                   # Robot description files
│       ├── agilex_piper_macro.urdf.xacro
│       ├── agilex_piper_single.system.xacro
│       ├── agilex_piper_dual.system.xacro
│       └── include/            # ros2_control configurations
└── socket_can_hardware_interface/  # Core CAN interface implementation
```

## Features

### 🤖 **Robot Control Modes**
- **Position Control**: Standard position-based joint control
- **MIT Control Mode**: Advanced torque control with current feedback
- **Mock Hardware**: Simulation mode for testing without physical hardware
- **Isaac Sim Integration**: Support for NVIDIA Isaac Sim
- **Gazebo Support**: Classic and modern Gazebo simulation

### 🔌 **CAN Communication**
- **Real-time CAN Interface**: Direct CAN bus communication via SocketCAN
- **Multi-joint Feedback**: Simultaneous monitoring of all 6 joints
- **High-speed Feedback**: Velocity and effort feedback at high frequencies
- **Status Monitoring**: Comprehensive arm status and error reporting
- **Configurable CAN IDs**: Flexible addressing scheme

### 🎯 **Hardware Interfaces**
- **Position Interface**: Joint position command and feedback
- **Velocity Interface**: Joint velocity feedback
- **Effort Interface**: Joint torque/effort feedback
- **Status Interface**: Robot arm status, control modes, and error codes

### 🏗️ **Multi-Robot Support**
- **Namespace Prefixing**: Support for multiple robot instances
- **Flexible Configuration**: Parameterized robot descriptions
- **Dual Robot Setup**: Pre-configured dual robot launch files

## Quick Start

### Prerequisites
- ROS 2 Humble or later
- Ubuntu 22.04 LTS
- CAN interface hardware (for real robot)
- MoveIt2 (for motion planning)

### Installation

1. **Clone the repository** (if not already in workspace):
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> can_hardware_interface
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:
   ```bash
   colcon build --packages-select can_data_plugins socket_can_hardware_interface piper_bringup
   source install/setup.bash
   ```

### Usage Examples

#### 🚀 **Launch with Mock Hardware (Simulation)**
```bash
ros2 launch piper_bringup piper_bringup.launch.py use_mock_hardware:=true
```

#### 🔧 **Launch with Real Hardware**
```bash
# Ensure CAN interface is up
sudo ip link set can1 up type can bitrate 1000000

# Launch with real hardware
ros2 launch piper_bringup piper_bringup.launch.py use_mock_hardware:=false
```

#### 🎮 **Launch with Prefix (Multi-robot)**
```bash
ros2 launch piper_bringup piper_bringup.launch.py use_mock_hardware:=true prefix:=robot1_
```

#### 🌐 **Launch with Isaac Sim**
```bash
ros2 launch piper_bringup piper_bringup.launch.py sim_isaac_sim:=true
```

#### 👥 **Dual Robot Setup**
```bash
ros2 launch piper_bringup piper_bringup_dual.launch.py use_mock_hardware:=true
```

## Configuration

### Launch Parameters

The main launch file supports the following parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_mock_hardware` | `false` | Use mock hardware for simulation |
| `mock_sensor_commands` | `false` | Enable mock sensor command interfaces |
| `sim_gazebo_classic` | `false` | Use classic Gazebo simulation |
| `sim_gazebo` | `false` | Use modern Gazebo simulation |
| `sim_isaac_sim` | `false` | Use NVIDIA Isaac Sim |
| `prefix` | `""` | Namespace prefix for multi-robot setups |
| `simulation_controllers` | `""` | Additional simulation controllers |

### CAN Interface Setup

For real hardware operation, configure your CAN interface:

```bash
# Load CAN modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_bcm

# Bring up CAN interface
sudo ip link set can1 up type can bitrate 1000000

# Verify interface
ip link show can1
candump can1  # Monitor CAN traffic
```

### Robot Description Configuration

The robot description supports flexible configuration through xacro parameters:

```xml
<!-- Basic usage -->
<xacro:agilex_piper name="agilex_piper" parent="base_link">
  <origin xyz="0 0 0" rpy="0 0 0" />
</xacro:agilex_piper>

<!-- With namespace prefix -->
<xacro:agilex_piper name="agilex_piper" parent="base_link" prefix="robot1_">
  <origin xyz="0 0 0" rpy="0 0 0" />
</xacro:agilex_piper>
```

## CAN Protocol Details

### 📡 **CAN Message Structure**

The Piper robot uses a structured CAN protocol with specific message IDs:

#### **Status Messages (Read-only)**
- `0x2A1` (673): Arm status, control mode, errors
- `0x2A5` (677): Joint 1&2 position feedback  
- `0x2A6` (678): Joint 3&4 position feedback
- `0x2A7` (679): Joint 5&6 position feedback

#### **Control Messages (Write)**
- `0x151` (337): Motion control configuration
- `0x155` (341): Joint 1&2 position commands
- `0x156` (342): Joint 3&4 position commands  
- `0x157` (343): Joint 5&6 position commands
- `0x471` (1137): Arm disable/enable configuration

### 🔧 **Control Modes**

Configure control mode in the ros2_control configuration:

```xml
<!-- Position-Speed Mode (Normal) -->
<param name="mit_mode">0</param>

<!-- MIT Mode (Advanced Torque Control) -->
<param name="mit_mode">173</param>
```

Control modes:
- `0x00`: Position-speed mode (default)
- `0xAD` (173): MIT mode for advanced torque control

### 🔢 **Data Encoding**

- **Position**: 32-bit integer, unit: millidegrees (divide by 1000 * 180/π for radians)
- **Velocity**: 16-bit integer, unit: millirad/s (divide by 1000 for rad/s)
- **Effort**: 16-bit integer with joint-specific scaling coefficients

## Plugin Architecture

The system uses a modular plugin architecture for CAN data processing:

### **Available Device Types**
- `PIPER`: Standard Piper robot arm communication
- `OMNI_PICKER`: Omni picker attachment support

### **Plugin Categories**
- **Feedback Plugins**: Handle incoming sensor data from robot
- **Control Plugins**: Send position/velocity commands to robot  
- **MIT Control Plugins**: Advanced torque control mode
- **Status Plugins**: Monitor system status and errors

## Hardware Interface Details

### **Exported Interfaces**

The hardware interface exports the following ROS 2 control interfaces:

#### **Joint Interfaces** (for each joint: joint1-joint6)
- `position` (command & state)
- `velocity` (state only)
- `effort` (state only)

#### **System Interfaces**
- `arm_status/*` - Robot arm status information
- `joint*_feedback/*` - Joint feedback data
- `arm_motion_ctrl2/*` - Motion control configuration

### **Multi-Robot Support**

When using the `prefix` parameter, all interfaces are prefixed:
- Joint names: `robot1_joint1`, `robot1_joint2`, etc.
- Hardware interfaces: `robot1_joint1/position`, etc.

## Troubleshooting

### Common Issues

1. **CAN Interface Not Found**
   ```bash
   # Check if CAN interface exists
   ip link show can1
   
   # If not, configure it
   sudo ip link set can1 up type can bitrate 1000000
   ```

2. **Permission Denied on CAN Interface**
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Re-login for changes to take effect
   ```

3. **Robot Not Responding**
   ```bash
   # Check CAN traffic
   candump can1
   
   # Send test frame
   cansend can1 151#0000000000000000
   ```

4. **Joint Limits Exceeded**
   - Check joint limit definitions in URDF
   - Verify command values are within acceptable ranges
   - Monitor robot status messages for error codes

### Debug Tools

```bash
# Monitor CAN traffic
candump can1

# Send raw CAN frames
cansend can1 <ID>#<DATA>

# Check ros2_control status
ros2 control list_hardware_interfaces

# Monitor joint states
ros2 topic echo /joint_states

# Check controller status
ros2 control list_controllers
```

## Development

### Adding New Plugins

1. Create plugin class inheriting from base interface
2. Implement required methods for CAN communication
3. Register plugin in CMakeLists.txt
4. Add device configuration to ros2_control URDF
