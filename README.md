# Real-Time Cartesian Controller & Vendor-Neutral PLC Integration Bridge for UR5/UR5e Robots

Cartesian position controller (`ros2_control` plugin) and vendor-neutral PLC integration bridge for ROS 2 industrial robots, run in simulation and on UR5/UR5e hardware. Features damped least-squares differential inverse kinematics driven by SE(3) pose feedback, and a configurable hardware interface supporting EtherNet/IP and OPC UA protocols for PLC-governed manufacturing cells.

## 🎯 Overview

This project implements a position-only Cartesian controller that drives the end effector of a UR5 or UR5e to a target position. A PID on the Cartesian position error produces a Cartesian velocity, and a damped pseudo-inverse of the position Jacobian maps it to joint velocity commands. Tool and target poses come from a swappable SE(3) pose source; in the current demos that source is emulated through TF.

In addition, the project includes a **vendor-neutral PLC integration bridge** (`plc_ros2_bridge`) that enables the same ROS 2 control architecture to deploy across heterogeneous industrial PLC environments — Allen-Bradley via EtherNet/IP, Siemens/Beckhoff via OPC UA — without code changes. This package is **under active development**.

### Key Features

- **Cartesian Position Control**: Position-only end-effector control with PID feedback at 100 Hz
- **SE(3) Pose Source**: Sensor hardware component that exports tool and target poses as state interfaces (emulated through TF in the demos)
- **Multiple Operation Modes**: Mock hardware, Gazebo simulation, and real robot through the same controller plugin
- **Dynamic Target Setting**: Runtime target position updates via TF transforms
- **Damped Least-Squares Inverse**: SVD-based damped pseudo-inverse for well-behaved commands near singularities
- **ros2_control Integration**: Native controller plugin with lifecycle management
- **Vendor-Neutral PLC Bridge** *(under development)*: Configurable `ros2_control` hardware interface supporting EtherNet/IP (Allen-Bradley) and OPC UA protocols
- **Safety Validation Layer** *(under development)*: State machine, watchdog, and joint limit enforcement for safe ROS 2 ↔ PLC handoff
- **YAML-Driven Tag Mapping** *(under development)*: Adapt to any PLC program without recompiling — change a config file, not code

## 🏗️ System Architecture

### Architecture 1 — Real-Time Cartesian Control

![System Architecture 1](docs/images/system_architecture.png)

Cartesian position control from SE(3) pose feedback and differential inverse kinematics:

1. **Pose Source** provides the tool and target poses (emulated through TF in the demos)
2. **TF Lookup Server** streams those poses over a TCP socket
3. **Sensor Hardware Component** receives them and exports them as `ros2_control` state interfaces
4. **Cartesian Controller** computes the position error, runs the PID, and maps the result to joint velocities
5. **UR5/UR5e Robot** executes the joint velocity commands

> **Current state:** the controller reads the tool and target poses through its own tf2 listener. Claiming the exported sensor state interfaces directly is not implemented yet.

### Architecture 2 — Full System with Vendor-Neutral PLC Integration *(Under Development)*

![System Architecture 2](docs/images/system_architecture_2.png)

Extends Architecture 1 with a PLC integration bridge, enabling the same Cartesian controller to deploy in PLC-governed manufacturing cells:

1. **Pose Source** provides the tool and target poses (emulated through TF in the demos)
2. **TF Lookup Server** streams those poses over a TCP socket
3. **Sensor Hardware Component** receives them and exports them as `ros2_control` state interfaces
4. **Cartesian Controller** computes the position error, runs the PID, and maps the result to joint velocities
5. **PLC Bridge** *(under development)* translates joint commands to PLC tags via EtherNet/IP or OPC UA
6. **Safety Validator** *(under development)* enforces joint limits, watchdog, and E-stop monitoring
7. **Industrial PLC** retains ultimate safety authority over robot drives
8. **UR5/UR5e Robot** executes the joint velocity commands

> **Note:** The Cartesian controller code is identical in both architectures. Only the hardware interface plugin changes — from direct robot communication to PLC-mediated control. This is the vendor-neutral, reusable pattern at the core of this project.

## 🎥 Live Demonstration

[![Watch the demo](https://img.youtube.com/vi/FevBLPXetxo/0.jpg)](https://www.youtube.com/watch?v=FevBLPXetxo)

*UR5 Cartesian position control with pose feedback emulated through TF. (Visualization)*

[![Watch the demo](https://img.youtube.com/vi/lPNE6-0R59k/0.jpg)](https://www.youtube.com/watch?v=lPNE6-0R59k)

*UR5 Cartesian position control with pose feedback emulated through TF. (Visualization and Simulation)*

[![Watch the demo](https://img.youtube.com/vi/UKBMwUgmN18/0.jpg)](https://www.youtube.com/shorts/UKBMwUgmN18) 

*Real robot demo using the UR5 - Will post the UR5e later*

### Package Organization

```
robotics_packages/
├── hardware_interfaces/
│   ├── se3_sensor_driver/          # SE3 sensor hardware interface
│   └── plc_ros2_bridge/            # Vendor-neutral PLC integration (under development)
│       ├── include/
│       │   └── plc_ros2_bridge/
│       │       ├── plc_interface.hpp           # Abstract vendor-neutral PLC interface
│       │       ├── ethernet_ip_interface.hpp    # EtherNet/IP (Allen-Bradley) implementation
│       │       ├── opcua_interface.hpp          # OPC UA (vendor-neutral) implementation
│       │       ├── safety_validator.hpp         # Safety validation state machine
│       │       ├── tag_registry.hpp             # ROS 2 ↔ PLC tag mapping
│       │       └── plc_hardware_interface.hpp   # ros2_control SystemInterface
│       ├── src/
│       ├── config/
│       ├── urdf/
│       └── doc/
├── controllers/
│   └── cartesian_controller/       # Core Cartesian control algorithms
└── applications/
    └── ur5e_cartesian_control/     # UR5e-specific implementation
```

## 🚀 Quick Start

### Prerequisites

- ROS 2 Humble
- Universal Robots ROS 2 driver
- KDL (Kinematics and Dynamics Library)
- Eigen3

### Installation

1. **Clone the repository**:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Seyi-roboticist/_controller_.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:
   ```bash
   colcon build --packages-select se3_sensor_driver ros2_controller_cartesian ur5e_cartesian_control
   source install/setup.bash
   ```

   To also build the PLC bridge *(under development)*:
   ```bash
   colcon build --packages-select plc_ros2_bridge
   ```

### Basic Usage

#### For a Real UR5 / UR5e Robot:
```bash
ros2 launch ur5e_cartesian_control ur5e.launch.py use_fake:=false ur_type:=ur5e
```
The launch file does not expose the robot IP yet. Set the `robot_ip` default in `urdf/ur5e_with_sensors.xacro` before launching.

#### For Gazebo Simulation:
```bash
ros2 launch ur5e_cartesian_control ur5e.launch.py use_fake:=true use_gazebo:=true
```

#### For Mock Hardware (no physics):
```bash
ros2 launch ur5e_cartesian_control ur5e.launch.py use_fake:=true
```

## 🎮 Dynamic Target Control

Set target positions dynamically during operation:

```bash
# Move to position (x=0.4, y=0.2, z=0.5)
ros2 run tf2_ros static_transform_publisher 0.4 0.2 0.5 0 0 0 base_link target_sensor_frame

# Automated two-target demo
ros2 run ur5e_cartesian_control two_positions_transform_monitor
```

## 🔧 Configuration

### Controller Parameters

Key parameters in `config/ur5e_controllers.yaml`:

```yaml
cartesian_controller:
  ros__parameters:
    velocity_scaling_factor: 0.07      # Global command scale (reduced for hardware bring-up)
    error_threshold: 0.005             # Stop threshold on FK position error (meters)
    position_gain: [2.2, 2.2, 2.2]    # Proportional gains [x,y,z]
    integral_gain: [0.02, 0.02, 0.02] # Integral gains [x,y,z]
    derivative_gain: [0.5, 0.5, 0.5]  # Derivative gains [x,y,z]
    damping_factor: 0.05               # Jacobian damping
```

The controller manager runs at `update_rate: 100` (Hz), set in the same file.

### Sensor Configuration

Configure SE3 sensors in your launch file:

```python
DeclareLaunchArgument("sensor_ip", default_value="192.168.1.100")
DeclareLaunchArgument("robot_sensor_name", default_value="robot_sensor")
DeclareLaunchArgument("target_sensor_name", default_value="target_sensor")
```

## 🔌 PLC Integration Bridge *(Under Active Development)*

The `plc_ros2_bridge` package provides a vendor-neutral `ros2_control` hardware interface that enables ROS 2 controllers to send commands to and receive feedback from industrial PLCs.

### Architecture

```
ROS 2 Controller → PLCHardwareInterface → SafetyValidator → PLCInterface → PLC → Robot
                   (ros2_control)          (defense-in-depth)  (EtherNet/IP    (safety
                                                                or OPC UA)     authority)
```

The Cartesian controller code is **unchanged** regardless of whether commands go directly to the robot or through a PLC — only the hardware interface plugin swaps. This is the reusable, vendor-neutral pattern that enables deployment across heterogeneous factory floors.

### Supported Protocols

| Protocol | Target PLCs | Status |
|----------|------------|--------|
| **EtherNet/IP (CIP)** | Allen-Bradley ControlLogix, CompactLogix | Reference implementation |
| **OPC UA** | Siemens S7-1500, Beckhoff TwinCAT 3, B&R, Omron | Structural implementation |
| **Modbus TCP** | Legacy systems | Planned |

### Key Components

- **`PLCInterface`** — Abstract vendor-neutral interface defining the read/write/connect contract for PLC communication
- **`EtherNetIPInterface`** — Reference implementation for Allen-Bradley controllers using CIP over TCP/IP
- **`OPCUAInterface`** — Vendor-neutral implementation for OPC UA-enabled controllers
- **`SafetyValidator`** — State machine (IDLE → OPERATIONAL → ESTOP → FAULT) with watchdog timer and joint limit enforcement
- **`TagRegistry`** — Bidirectional ROS 2 ↔ PLC tag mapping loaded from YAML configuration
- **`PLCHardwareInterface`** — The `ros2_control::SystemInterface` that ties everything together

### PLC Bridge Configuration *(Under Development)*

Tag mappings are defined in YAML, enabling deployment to different PLC programs without code changes:

```yaml
tag_mappings:
  - ros2_interface: "shoulder_pan_joint/velocity"
    plc_tag: "Program:MainProgram.ROS_JointVelCmd[0]"
    direction: "to_plc"
    safety_class: "command"

  - ros2_interface: "shoulder_pan_joint/position"
    plc_tag: "Program:MainProgram.ROS_JointPosFb[0]"
    direction: "from_plc"
    safety_class: "status"

watchdog:
  heartbeat_tag: "Program:MainProgram.ROS_Heartbeat"
  timeout_ms: 500
```

### Safety Methodology

The PLC retains ultimate safety authority at all times. The `SafetyValidator` provides defense-in-depth from the ROS 2 side — it validates commands before they reach the PLC but does not replace PLC-based safety functions running on SIL-rated hardware.

For the complete safety methodology, see [`doc/SAFETY_METHODOLOGY.md`](robotics_packages/hardware_interfaces/plc_ros2_bridge/doc/SAFETY_METHODOLOGY.md).

### Development Roadmap

- [x] Abstract `PLCInterface` with vendor-neutral contract
- [x] EtherNet/IP (CIP) reference implementation
- [x] OPC UA structural implementation
- [x] Safety validation interface and state machine design
- [x] YAML-driven tag registry
- [x] `ros2_control` SystemInterface integration
- [ ] Integration testing with Allen-Bradley CompactLogix hardware
- [ ] OPC UA integration with open62541 stack
- [ ] Modbus TCP implementation
- [ ] End-to-end validation with UR5e through PLC

## 📊 Technical Details

### Control Algorithm

1. **Pose Acquisition**: Tool and target poses are read in the base frame (through TF in the demos)
2. **Error Calculation**: Cartesian position error, target minus tool
3. **PID**: The position error produces a Cartesian velocity command
4. **Differential Inverse Kinematics**: SVD-based damped pseudo-inverse of the 3×6 position Jacobian, followed by a global velocity scale
5. **Joint Control**: Joint velocity commands are written to the command interfaces
6. **Stop Condition**: Zero velocity is commanded once the FK position error, computed from measured joint positions, is within the threshold

### Operating Parameters and Validation Status

- **Controller Update Rate**: 100 Hz (`controller_manager` `update_rate`)
- **Pose Server Rate**: 500 Hz (2 ms timer in `tf_lookup_server`)
- **Stop Threshold**: 5 mm FK position error. This is a stopping criterion, not a measured absolute accuracy
- **Velocity Scaling**: 0.07 on hardware
- **Run On**: mock hardware, Ignition Gazebo, and physical UR5 and UR5e arms
- **Endpoint Check on Hardware**: coarse ruler measurement only; no independent accuracy or repeatability measurement yet

### Current Limitations (v1)

- Position only; orientation is not controlled
- The controller reads poses through tf2 rather than claiming the sensor state interfaces
- Missing or stale pose data is not yet treated as a fault
- No per-joint velocity or position limits in the controller; only the global velocity scale
- `update()` is not yet hard real-time safe (TF lookups, dynamic allocation, and logging inside the loop)

## 🧪 Testing & Validation

The controller has been exercised in three environments:

- **Mock hardware**: interface claiming and lifecycle behavior, without physics
- **Gazebo simulation**: convergence and tuning
- **Real hardware**: UR5 and UR5e, multi-target tracking

There are no automated unit or integration tests in the repository yet.

## 🔍 Monitoring & Debugging

### RViz Visualization
Launch with visualization:
```bash
ros2 launch ur5e_cartesian_control ur5e.launch.py use_rviz:=true
```

### Debug Information
The controller logs the sensor error, FK error, and joint velocity commands to the console once per second. To inspect controller and interface state:
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Universal Robots for the UR robot platform
- ROS 2 Control framework contributors
- KDL library maintainers
- Johns Hopkins University, Robot System Programming (EN.530.707)

## 📞 Support

For questions, issues, or contributions:
- **Issues**: [GitHub Issues](https://github.com/Seyi-roboticist/_controller_/issues)
- **Email**: Contact through GitHub profile

---

*Built with ❤️ for the robotics community*
