# SE3 Sensor Driver

Hardware interface for SE3 sensors in ROS 2 Control framework.

## Overview

This package provides a sensor-only hardware interface that reads 3D pose data from external SE3 sensors via TCP/IP communication.

## Features

- Real-time pose data acquisition
- TF frame publishing
- Socket-based communication
- Multi-sensor support

## Usage

```bash
ros2 launch se3_sensor_driver se3_sensor.launch.py
```

## Configuration

See main project README for full configuration details.

---

Part of the [Cartesian Controller Project](../README.md)
