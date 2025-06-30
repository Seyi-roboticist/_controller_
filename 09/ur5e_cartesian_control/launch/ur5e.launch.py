from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("x", default_value="0.5"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.5"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        DeclareLaunchArgument("use_fake", default_value="false"),
        DeclareLaunchArgument("sensor_ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("robot_sensor_name", default_value="robot_sensor"),
        DeclareLaunchArgument("robot_frame_id", default_value="robot_sensor_frame"),
        DeclareLaunchArgument("target_sensor_name", default_value="target_sensor"),
        DeclareLaunchArgument("target_frame_id", default_value="target_sensor_frame"),
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("robot_ip", default_value="172.22.22.2"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("runtime_config_package", default_value="ur5e_cartesian_control"),
        DeclareLaunchArgument("controllers_file", default_value="ur5e_controllers.yaml"),
        DeclareLaunchArgument("description_package", default_value="ur5e_cartesian_control"),
        DeclareLaunchArgument("description_file", default_value="ur5e_with_sensors.xacro"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="10"),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller"),
        DeclareLaunchArgument("activate_joint_controller", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="true"),
        DeclareLaunchArgument("launch_dashboard_client", default_value="false"),
        DeclareLaunchArgument("use_tool_communication", default_value="false"),
        DeclareLaunchArgument("tool_parity", default_value="0"),
        DeclareLaunchArgument("tool_baud_rate", default_value="115200"),
        DeclareLaunchArgument("tool_stop_bits", default_value="1"),
        DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5"),
        DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5"),
        DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR"),
        DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
        DeclareLaunchArgument("tool_voltage", default_value="0"),
        DeclareLaunchArgument("reverse_ip", default_value="172.22.22.10"),
        DeclareLaunchArgument("script_command_port", default_value="50004"),
        DeclareLaunchArgument("reverse_port", default_value="50001"),
        DeclareLaunchArgument("script_sender_port", default_value="50002"),
        DeclareLaunchArgument("trajectory_port", default_value="50003"),
        DeclareLaunchArgument("sim_ignition", default_value="false"),
        DeclareLaunchArgument("sim_isaac", default_value="false"),
        DeclareLaunchArgument("com_port", default_value="/dev/ttyUSB0"),
     
        DeclareLaunchArgument(
            "world_path", 
            default_value=PathJoinSubstitution([FindPackageShare("ur5e_cartesian_control"), "world", "ur5e_world.sdf"])
        ),

        DeclareLaunchArgument(
            "script_filename", 
            default_value=PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "external_control.urscript"])
        ),

        DeclareLaunchArgument(
            "joint_limits_parameters_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5", "joint_limits.yaml"])
        ),
        DeclareLaunchArgument(
            "kinematics_parameters_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5", "default_kinematics.yaml"]),
            description="The calibration configuration of the actual robot used."
        ),
        DeclareLaunchArgument(
            "physical_parameters_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5", "physical_parameters.yaml"])
        ),
        DeclareLaunchArgument(
            "visual_parameters_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5", "visual_parameters.yaml"])
        )
    ]

    # Initializations
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_fake= LaunchConfiguration("use_fake")
    sensor_ip = LaunchConfiguration("sensor_ip")
    controllers_file = LaunchConfiguration("controllers_file")
    robot_sensor_name = LaunchConfiguration("robot_sensor_name")
    robot_frame_id = LaunchConfiguration("robot_frame_id")
    target_sensor_name = LaunchConfiguration("target_sensor_name")
    target_frame_id = LaunchConfiguration("target_frame_id")
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    joint_limits_parameters_file = LaunchConfiguration("joint_limits_parameters_file")
    kinematics_parameters_file = LaunchConfiguration("kinematics_parameters_file")
    physical_parameters_file = LaunchConfiguration("physical_parameters_file")
    visual_parameters_file = LaunchConfiguration("visual_parameters_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")
    sim_ignition = LaunchConfiguration("sim_ignition")
    sim_isaac = LaunchConfiguration("sim_isaac")
    com_port = LaunchConfiguration("com_port")
    world_path = LaunchConfiguration("world_path")
    script_filename = LaunchConfiguration("script_filename")

    pkg_path = FindPackageShare("ur5e_cartesian_control").find("ur5e_cartesian_control")
    ur_description_path = FindPackageShare("ur_description").find("ur_description")
    initial_positions = PathJoinSubstitution([pkg_path, "config", "initial_positions.yaml"])
    controllers_config = PathJoinSubstitution([pkg_path, "config", controllers_file])
    rviz_config_file = PathJoinSubstitution([pkg_path, "rviz", "ur5e_cartesian.rviz"])
    xacro_file = PathJoinSubstitution([pkg_path, "urdf", "ur5e_with_sensors.xacro"])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file,
        " sim_ignition:=", sim_ignition,
        " use_fake_hardware:=", use_fake,
        " ur_type:=", ur_type,
        " joint_limits_parameters_file:=", joint_limits_parameters_file,
        " kinematics_parameters_file:=", kinematics_parameters_file,
        " physical_parameters_file:=", physical_parameters_file,
        " visual_parameters_file:=", visual_parameters_file,
        " sensor_ip:=", sensor_ip,
        " robot_sensor_name:=", robot_sensor_name,
        " robot_frame_id:=", robot_frame_id,
        " target_sensor_name:=", target_sensor_name,
        " target_frame_id:=", target_frame_id,
        " initial_positions_file:=", initial_positions,
        " simulation_controllers:=", controllers_config,
        " tf_prefix:=", tf_prefix
    ])
    robot_description = {"robot_description": robot_description_content}

    # Base Launch 
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"]),
        launch_arguments={
            "ur_type": ur_type, 
            "robot_ip": robot_ip,
            "safety_limits": safety_limits,
            "safety_k_position": safety_k_position,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "kinematics_parameters_file": kinematics_parameters_file,
            "joint_limits_parameters_file": joint_limits_parameters_file,
            "physical_parameters_file": physical_parameters_file,
            "visual_parameters_file": visual_parameters_file,
            "tf_prefix": tf_prefix,
            "use_fake_hardware": use_fake,
            "fake_sensor_commands": fake_sensor_commands,
            "controller_spawner_timeout": controller_spawner_timeout,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "headless_mode": headless_mode,
            "launch_rviz": launch_rviz,
            "launch_dashboard_client": launch_dashboard_client,
            "use_tool_communication": use_tool_communication,
            "tool_parity": tool_parity,
            "tool_baud_rate": tool_baud_rate,
            "tool_stop_bits": tool_stop_bits,
            "tool_rx_idle_chars": tool_rx_idle_chars, 
            "tool_tx_idle_chars": tool_tx_idle_chars, 
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
            "tool_voltage": tool_voltage,
            "reverse_ip": reverse_ip,
            "script_command_port": script_command_port,
            "reverse_port": reverse_port,
            "script_sender_port": script_sender_port,
            "trajectory_port": trajectory_port,
            "script_filename": script_filename
        }.items(),
    )

    static_transform_target = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_target",
        namespace="ur5e_robot", 
        arguments=[x, y, z, "0", "0", "0", "base_link", target_frame_id],
    )
    
    static_transform_sensor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_sensor",
        namespace="ur5e_robot", 
        arguments=["0", "0", "0", "0", "0", "0", "tool0", robot_frame_id],
    )
    
    static_transform_world_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_world_base",
        namespace="ur5e_robot",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    tf_lookup_server = Node(
        package="se3_sensor_driver",
        executable="tf_lookup_server",
        name="tf_lookup_server",
        namespace="ur5e_robot", 
        parameters=[{
            "robot_parent_frame": "tool0",
            "robot_sensor_frame": robot_frame_id,
            "target_parent_frame": "base_link",
            "target_sensor_frame": target_frame_id
        }],
        output="screen",
    )
    
    cartesian_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_controller", "-c", "/controller_manager"],
        name="spawner_cartesian_controller",  
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": ["-r -v 3 ", world_path]}.items(),
        condition=IfCondition(use_gazebo),
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
        base_launch,
        tf_monitor_node,
        static_transform_world_base,
        #static_transform_target,
        static_transform_sensor,
        tf_lookup_server,
        cartesian_controller_spawner,
        rviz_node
    ])