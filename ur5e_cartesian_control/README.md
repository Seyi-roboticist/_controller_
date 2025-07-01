# My Legendary UR5e Cartesian Control

This package implements a Cartesian control system for the UR5e robot using external SE3 sensors for position tracking.

## Dynamic Target Setting

I found out that you can publish a new target dynamically by using the static transform publisher command:

```bash
ros2 run tf2_ros static_transform_publisher -0.5 -0.3 0.6 0 0 0 world target_sensor_frame
```

This allows me to change the target position of the robot on-the-fly without needing to restart the system all the time you know. The command sets a target at position x=-0.5, y=-0.3, z=0.6 with no rotation, using "world" as the reference frame. If time permits, I will write a script for this to automate this. 

**The Demo required us ultimately writing a script and I accomplished this before the assignment was posted. I will make it neater later but it works great now**

For real robot:
```bash
ros2 launch ur5e_cartesian_control ur5e.launch.py
```
The code works perfectly in all three modes. Please make sure you use `ur5e.launch.py` for all three modes as the other launch file is experimental. Thanks
