# Vendor-Neutral ROS 2 ↔ PLC Integration Bridge

A ros2_control-compliant hardware interface that enables ROS 2 adaptive robotics controllers to interoperate with legacy PLC-governed industrial control systems through a vendor-neutral abstraction layer.

## Why This Exists

Modern robotics frameworks (ROS 2, MoveIt 2) and industrial control systems (Allen-Bradley, Siemens PLCs) were not designed to interoperate. This structural incompatibility forces U.S. manufacturers to build bespoke integration for every deployment — expensive, risky, and non-reusable.

This package provides the missing infrastructure: a standardized bridge that any system integrator can deploy without proprietary licensing, vendor lock-in, or custom engineering per installation.

## Architecture

```
ROS 2 Controller ──→ PLCHardwareInterface ──→ SafetyValidator ──→ PLCInterface ──→ PLC ──→ Robot
                     (ros2_control)            (defense-in-depth)   (EtherNet/IP     (safety
                                                                     or OPC UA)       authority)
```

**Key design decisions:**
- **Vendor-neutral:** Abstract `PLCInterface` with protocol-specific implementations
- **Safety-first:** PLC retains ultimate authority; ROS 2 provides advisory setpoints
- **Configuration over code:** YAML tag mapping adapts to any PLC program
- **ros2_control native:** Works with any existing controller (Cartesian, MoveIt, trajectory)

See [doc/ARCHITECTURE.md](doc/ARCHITECTURE.md) for the full system design.

## Supported Protocols

| Protocol | Target PLCs | Status |
|----------|------------|--------|
| **EtherNet/IP** | Allen-Bradley ControlLogix, CompactLogix | Reference implementation |
| **OPC UA** | Siemens, Beckhoff, B&R, Omron, CODESYS | Structural implementation |

## Quick Start

### Prerequisites

- ROS 2 Humble
- ros2_control framework
- Network access to target PLC

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/Seyi-roboticist/_controller_.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select plc_ros2_bridge
source install/setup.bash
```

### Configuration

1. **Create a tag mapping file** based on your PLC program (see `config/tag_mapping.yaml` for an example UR5e configuration)

2. **Add the hardware interface to your robot URDF:**

```xml
<ros2_control name="PLCBridge" type="system">
  <hardware>
    <plugin>plc_ros2_bridge/PLCHardwareInterface</plugin>
    <param name="protocol">ethernet_ip</param>
    <param name="ip_address">192.168.1.10</param>
    <param name="slot">0</param>
    <param name="tag_mapping_file">config/tag_mapping.yaml</param>
  </hardware>
  <joint name="shoulder_pan_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... more joints ... -->
</ros2_control>
```

3. **Launch with your existing controllers** — no controller changes needed.

## Safety

This package implements a defense-in-depth safety validation layer. The PLC always retains ultimate safety authority. See [doc/SAFETY_METHODOLOGY.md](doc/SAFETY_METHODOLOGY.md) for the complete methodology.

## Integration with Cartesian Controller

This bridge is designed to work with the [Cartesian Controller](../controllers/cartesian_controller/) in this repository. The same controller code works whether controlling the robot directly or through a PLC — only the hardware interface plugin changes.

## License

MIT License — See [LICENSE](../LICENSE) for details.

## Author

Seyi R. Afolayan — Johns Hopkins University / The RDI Group
