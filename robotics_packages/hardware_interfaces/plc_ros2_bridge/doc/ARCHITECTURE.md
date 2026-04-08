# Architecture: Vendor-Neutral ROS 2 ↔ PLC Integration Bridge

**Author:** Seyi R. Afolayan  
**Date:** 2025  
**License:** MIT

## The Problem

Advanced robotics capabilities — adaptive motion planning, force-controlled assembly,
vision-guided manipulation — are routinely demonstrated in research laboratories. Yet
these capabilities remain largely undeployable in American manufacturing facilities.

The barrier is not the robotics technology itself. It is a **structural incompatibility**
between two control paradigms:

1. **ROS 2 (Robot Operating System 2):** The dominant open-source robotics middleware
   framework. Provides sophisticated algorithms for perception, planning, and adaptive
   control. Operates in soft real-time on general-purpose computing hardware.

2. **Industrial PLCs (Programmable Logic Controllers):** The established controllers
   governing manufacturing operations. Allen-Bradley (Rockwell Automation) and Siemens
   dominate the North American market. PLCs enforce deterministic cycle times, safety
   compliance (IEC 61508, ISO 13849), and process reliability. Every production robot
   cell in U.S. manufacturing operates under PLC supervision.

These systems were not designed to interoperate. ROS 2 cannot natively communicate with
PLC tag databases. PLCs cannot execute ROS 2 control algorithms. The result: manufacturers
who want adaptive robotics must hire specialized integrators to build **bespoke bridges**
for each deployment, at high cost and risk, with no reusable infrastructure.

## This Solution

This package — `plc_ros2_bridge` — provides the **missing integration layer**. It is a
ros2_control-compliant hardware interface that enables any ROS 2 controller to send
commands to and receive feedback from any PLC, through a vendor-neutral abstraction.

### Design Principles

1. **Vendor Neutrality:** The abstract `PLCInterface` class defines a common contract
   for tag read/write operations. Protocol-specific implementations (EtherNet/IP for
   Allen-Bradley, OPC UA for Siemens/Beckhoff/B&R) inherit from this interface. System
   integrators select the protocol via URDF configuration — no code changes required.

2. **Safety-First Architecture:** The PLC retains ultimate authority over safety-critical
   functions. The `SafetyValidator` layer ensures the ROS 2 side cannot inadvertently
   bypass PLC safety interlocks. This is defense-in-depth, not a replacement for
   certified safety logic.

3. **ros2_control Compliance:** The bridge implements `hardware_interface::SystemInterface`,
   making it compatible with the entire ros2_control ecosystem. Any existing controller —
   joint trajectory, Cartesian, force/torque, MoveIt 2 — works without modification.

4. **Configuration Over Code:** The `TagRegistry` maps ros2_control interface names to
   PLC tag addresses via YAML configuration. Adapting the bridge to a different PLC
   program or robot requires changing a config file, not recompiling.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Control Stack                         │
│                                                                     │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐  │
│  │   Cartesian       │  │  Joint Trajectory │  │  MoveIt 2       │  │
│  │   Controller      │  │  Controller       │  │  Servo          │  │
│  └────────┬─────────┘  └────────┬─────────┘  └────────┬────────┘  │
│           │ velocity cmd        │ position cmd         │           │
│           └─────────────┬───────┘                      │           │
│                         ▼                              │           │
│  ┌──────────────────────────────────────────────────────┘           │
│  │         ros2_control Controller Manager                         │
│  └──────────────────────┬──────────────────────────────────────────┘
│                         │ command/state interfaces
├─────────────────────────┼──────────────────────────────────────────┤
│                         ▼                                          │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │              PLCHardwareInterface                            │  │
│  │              (ros2_control SystemInterface)                  │  │
│  │                                                              │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │  │
│  │  │ TagRegistry  │  │ Safety      │  │ PLCInterface        │ │  │
│  │  │ (YAML-based  │  │ Validator   │  │ (abstract)          │ │  │
│  │  │  mapping)    │  │ (state      │  │                     │ │  │
│  │  │              │  │  machine,   │  │  ┌───────────────┐  │ │  │
│  │  │ ROS2 name    │  │  watchdog,  │  │  │ EtherNet/IP   │  │ │  │
│  │  │ ←→ PLC tag   │  │  limits)    │  │  │ (Allen-       │  │ │  │
│  │  │              │  │             │  │  │  Bradley)     │  │ │  │
│  │  └─────────────┘  └─────────────┘  │  ├───────────────┤  │ │  │
│  │                                     │  │ OPC UA        │  │ │  │
│  │                                     │  │ (Siemens,     │  │ │  │
│  │                                     │  │  Beckhoff,    │  │ │  │
│  │                                     │  │  B&R, etc.)   │  │ │  │
│  │                                     │  └───────────────┘  │ │  │
│  │                                     └─────────────────────┘ │  │
│  └──────────────────────────────┬───────────────────────────────┘  │
│                     plc_ros2_bridge                                 │
├─────────────────────────────────┼──────────────────────────────────┤
│                                 │ EtherNet/IP (TCP 44818)          │
│                                 │ or OPC UA (TCP 4840)             │
│                                 ▼                                  │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                    Industrial PLC                            │  │
│  │                    (Safety Authority)                        │  │
│  │                                                              │  │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌────────────┐  │  │
│  │  │ Safety Program   │  │ Motion Control   │  │ I/O Scan   │  │  │
│  │  │ - E-stop         │  │ - Drive commands │  │ - Encoders │  │  │
│  │  │ - Light curtains │  │ - Velocity limits│  │ - Sensors  │  │  │
│  │  │ - Zone monitor   │  │ - Servo enables  │  │ - Outputs  │  │  │
│  │  └─────────────────┘  └─────────────────┘  └────────────┘  │  │
│  │                                                              │  │
│  │         Allen-Bradley CompactLogix / ControlLogix            │  │
│  │         Siemens S7-1500 / Beckhoff TwinCAT 3                │  │
│  └──────────────────────────────┬───────────────────────────────┘  │
│                                 │ EtherNet/IP I/O                  │
│                                 │ or PROFINET / EtherCAT           │
│                                 ▼                                  │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │              Industrial Robot / Servo Drives                 │  │
│  │              (UR5e, Fanuc, ABB, KUKA, etc.)                 │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## Data Flow

### Control Cycle (Write Path)

1. ROS 2 controller (e.g., Cartesian controller) computes joint velocity commands
2. Controller Manager writes values to command interfaces
3. `PLCHardwareInterface::write()` is called:
   a. Reads command interface values
   b. Looks up PLC tag names via `TagRegistry`
   c. Validates commands through `SafetyValidator` (range checks, state verification)
   d. Applies unit conversion (scale/offset) if needed
   e. Batch-writes to PLC via `PLCInterface` (EtherNet/IP or OPC UA)
4. PLC receives setpoints, applies its own safety validation, commands drives

### Feedback Cycle (Read Path)

1. PLC reads encoder positions, computes velocities, monitors safety I/O
2. `PLCHardwareInterface::read()` is called:
   a. Looks up feedback tag names via `TagRegistry`
   b. Batch-reads from PLC via `PLCInterface`
   c. Applies inverse unit conversion
   d. Updates state interface values
   e. Reads safety tags (E-stop, watchdog) and feeds `SafetyValidator`
3. Controller Manager provides updated state to active controllers

## Safety Methodology

### Guiding Principle

> The PLC is the safety controller of record. This bridge does not implement safety
> logic — it enforces communication integrity so that the PLC's safety logic is
> never bypassed by the ROS 2 layer.

### Defense-in-Depth Layers

| Layer | Responsibility | Authority |
|-------|---------------|-----------|
| PLC Safety Program | E-stop, zone monitoring, speed limits, SIL-rated functions | **Ultimate** — runs on certified hardware |
| PLC Motion Control | Drive enables, velocity clamping, position limits | PLC program logic |
| SafetyValidator (this package) | Pre-flight validation of ROS 2 commands, watchdog monitoring | Advisory — blocks writes, cannot override PLC |
| ROS 2 Controller | Adaptive setpoint generation | Lowest — provides suggestions, not commands |

### Watchdog Protocol

```
ROS 2 Side                          PLC Side
──────────                          ────────
Every cycle:                        Continuous scan:
  Write heartbeat_tag++               Monitor heartbeat_tag
                                       If no change in timeout_ms:
                                         → Zero all robot outputs
                                         → Set fault flag
                                         → Require operator reset

If PLC watchdog_status_tag == false:
  → SafetyValidator enters FAULT
  → All writes blocked
  → Log error
```

### State Machine

```
   ┌──────────────┐
   │ INITIALIZING │ ──(startup complete)──→ ┌──────┐
   └──────────────┘                         │ IDLE │
                                            └──┬───┘
                                   (enable)    │
                                            ┌──▼──────────┐
                                  ┌────────→│ OPERATIONAL │←────────┐
                                  │         └──┬──────┬───┘         │
                              (resume)   (hold)│      │(E-stop)  (resume)
                                  │         ┌──▼──┐   │            │
                                  └─────────│HOLD │   │            │
                                            └─────┘   │            │
                                                   ┌──▼───┐        │
                                                   │ESTOP │        │
                                                   └──┬───┘        │
                                          (released)  │            │
                                            ┌─────────▼──────────┐ │
                                            │  RESET_REQUIRED    │ │
                                            └─────────┬──────────┘ │
                                           (operator  │            │
                                            reset)    └────────────┘
                                                       (via IDLE)
   ANY state ──(watchdog timeout / PLC fault)──→ ┌───────┐
                                                 │ FAULT │
                                                 └───────┘
```

## Integration with Existing Cartesian Controller

This bridge is designed to work with the Cartesian controller already in this repository.
The integration replaces the direct sensor → hardware path with a PLC-mediated path:

**Before (direct hardware):**
```
SE3 Sensor → TF Lookup → Cartesian Controller → ros2_control → UR Driver → Robot
```

**After (PLC-mediated):**
```
SE3 Sensor → TF Lookup → Cartesian Controller → ros2_control → PLC Bridge → PLC → Robot
```

The Cartesian controller code is unchanged. Only the URDF hardware plugin changes:
from `ur_robot_driver/URPositionHardwareInterface` to `plc_ros2_bridge/PLCHardwareInterface`.

This is the architectural pattern that enables reusable deployment: the same control
algorithms work whether the robot is a UR5e on Allen-Bradley, a Fanuc on Siemens, or
a KUKA on Beckhoff. The bridge handles the translation.

## Supported Protocols

### EtherNet/IP (Reference Implementation)

- **Target PLCs:** Allen-Bradley ControlLogix, CompactLogix, Micro800
- **Protocol:** CIP over TCP/IP (IEEE 802.3)
- **Port:** 44818
- **Addressing:** Symbolic tag names (e.g., `Program:MainProgram.ROS_JointVelCmd[0]`)
- **Batch I/O:** CIP Multiple Service Packet for reduced latency
- **Market relevance:** ~60% of North American manufacturing PLC installations

### OPC UA (Vendor-Neutral Implementation)

- **Target PLCs:** Siemens S7-1500, Beckhoff TwinCAT 3, B&R, Omron NX/NJ, CODESYS-based
- **Protocol:** OPC UA Binary (IEC 62541)
- **Port:** 4840
- **Addressing:** NodeId (namespace + identifier)
- **Batch I/O:** Native multi-node Read/Write services
- **Market relevance:** Supported by all major PLC vendors as the convergence standard

## References

- IEC 61131-3: PLC programming languages
- IEC 62541: OPC Unified Architecture
- IEEE 802.3: Ethernet standard
- CIP Specification (ODVA): Common Industrial Protocol
- ROS 2 Control documentation: https://control.ros.org
- IEC 61508: Functional safety of electrical/electronic systems
- ISO 13849: Safety of machinery — Safety-related parts of control systems
