from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("x", default_value="0.5", description="X position of the target"),
        DeclareLaunchArgument("y", default_value="0.0", description="Y position of the target"),
        DeclareLaunchArgument("z", default_value="0.5", description="Z position of the target"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz"),
        DeclareLaunchArgument("use_gazebo", default_value="false", description="Use Ignition Gazebo"),
        DeclareLaunchArgument("use_fake", default_value="false", description="Use fake hardware"),
        DeclareLaunchArgument("sensor_ip", default_value="192.168.1.100", description="Sensor IP address"),
        DeclareLaunchArgument("controllers_file", default_value="ur5e_controllers.yaml", description="Controllers config file"),
        DeclareLaunchArgument("robot_sensor_name", default_value="robot_sensor"),
        DeclareLaunchArgument("robot_frame_id", default_value="robot_sensor_frame"),
        DeclareLaunchArgument("target_sensor_name", default_value="target_sensor"),
        DeclareLaunchArgument("target_frame_id", default_value="target_sensor_frame"),
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("world_path", default_value="empty.sdf", description="Ignition world file"),
    ]

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_fake = LaunchConfiguration("use_fake")
    sensor_ip = LaunchConfiguration("sensor_ip")
    controllers_file = LaunchConfiguration("controllers_file")
    robot_sensor_name = LaunchConfiguration("robot_sensor_name")
    robot_frame_id = LaunchConfiguration("robot_frame_id")
    target_sensor_name = LaunchConfiguration("target_sensor_name")
    target_frame_id = LaunchConfiguration("target_frame_id")
    ur_type = LaunchConfiguration("ur_type")
    world_path = LaunchConfiguration("world_path")

    pkg_path = FindPackageShare("ur5e_cartesian_control").find("ur5e_cartesian_control")
    ur_description_path = FindPackageShare("ur_description").find("ur_description")
    initial_positions = PathJoinSubstitution([pkg_path, "config", "initial_positions.yaml"])
    controllers_config = PathJoinSubstitution([pkg_path, "config", controllers_file])
    rviz_config_file = PathJoinSubstitution([pkg_path, "rviz", "ur5e_cartesian.rviz"])
    xacro_file = PathJoinSubstitution([pkg_path, "urdf", "ur5e_with_sensors.xacro"])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file,
        " sim_ignition:=", use_gazebo,
        " use_fake_hardware:=", use_fake,
        " ur_type:=", ur_type,
        " sensor_ip:=", sensor_ip,
        " robot_sensor_name:=", robot_sensor_name,
        " robot_frame_id:=", robot_frame_id,
        " target_sensor_name:=", target_sensor_name,
        " target_frame_id:=", target_frame_id,
        " initial_positions_file:=", initial_positions,
        " simulation_controllers:=", controllers_config
    ])
    robot_description = {"robot_description": robot_description_content}

    static_transform_target = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_target",
        arguments=[x, y, z, "0", "0", "0", "base_link", target_frame_id],
    )
    static_transform_sensor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_sensor",
        arguments=["0", "0", "0", "0", "0", "0", "tool0", robot_frame_id],
    )
    static_transform_world_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_world_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    tf_lookup_server = Node(
        package="se3_sensor_driver",
        executable="tf_lookup_server",
        name="tf_lookup_server",
        parameters=[{
            "robot_parent_frame": "tool0",
            "robot_sensor_frame": robot_frame_id,
            "target_parent_frame": "base_link",
            "target_sensor_frame": target_frame_id
        }],
        output="screen",
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_gazebo}],
        output="screen",
    )

    joint_state_broadcaster_spawner_fake = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_gazebo),
    )
    
    joint_state_broadcaster_spawner_gazebo = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=IfCondition(use_gazebo),
    )
    
    cartesian_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_controller", "-c", "/controller_manager"],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": "-r -v 3"}.items(),
        condition=IfCondition(use_gazebo),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config, {"use_sim_time": use_gazebo}],
        output="screen",
        condition=UnlessCondition(use_gazebo),
    )

    robot_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", robot_description_content, "-name", "ur5e"],
        output="screen",
        parameters=[{"use_sim_time": use_gazebo}],
        condition=IfCondition(use_gazebo),
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock]"],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    tf_monitor_node = Node(
        package="ur5e_cartesian_control", 
        executable="two_positions_transform_monitor", 
        name="tf_monitor",
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_gazebo}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declared_arguments + [
        robot_state_pub,
        tf_monitor_node,
        static_transform_target,
        static_transform_sensor,
        static_transform_world_base,
        tf_lookup_server,
        gazebo_launch,
        robot_spawn,
        gz_sim_bridge,
        control_node,
        joint_state_broadcaster_spawner_fake,
        joint_state_broadcaster_spawner_gazebo,
        cartesian_controller_spawner,
        rviz_node,
    ])