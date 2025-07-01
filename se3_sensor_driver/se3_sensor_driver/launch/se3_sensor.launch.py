from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Declaring launch arguments just like we have done in other assignments 
    declared_arguments = [
        DeclareLaunchArgument("sensor_ip", default_value="192.168.1.100", description="IP address for the sensor system"), 
        DeclareLaunchArgument("robot_sensor_name", default_value="robot_sensor", description="Name of the robot"),
        DeclareLaunchArgument("robot_frame_id", default_value="robot_sensor_frame", description="Frame ID for the robot sensor"),
        DeclareLaunchArgument("target_sensor_name", default_value="target_sensor", description="Name of the target sensor"),
        DeclareLaunchArgument("target_frame_id", default_value="target_sensor_frame", description="Frame ID for the target sensor")
    ]

    # Get the launch configurations
    sensor_ip = LaunchConfiguration("sensor_ip")
    robot_sensor_name = LaunchConfiguration("robot_sensor_name")
    robot_frame_id = LaunchConfiguration("robot_frame_id")
    target_sensor_name = LaunchConfiguration("target_sensor_name")
    target_frame_id = LaunchConfiguration("target_frame_id")
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), 
            " ", 
            PathJoinSubstitution([FindPackageShare("se3_sensor_driver"), "urdf", "se3_sensor.urdf.xacro"]), 
            " ", 
            "sensor_ip:=", 
            sensor_ip,
            " ", 
            "robot_sensor_name:=", 
            robot_sensor_name,
            " ", 
            "robot_frame_id:=", 
            robot_frame_id,
            " ", 
            "target_sensor_name:=", 
            target_sensor_name,
            " ", 
            "target_frame_id:=", 
            target_frame_id
        ]
    )

    # Processed URDF with changeable parameters already applied
    robot_description = {"robot_description": robot_description_content}

    # YAML file path 
    se3_controllers = PathJoinSubstitution(
        [FindPackageShare("se3_sensor_driver"), "config", "se3_sensor.yaml"]
    )

    # RViz file path 
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("se3_sensor_driver"), "rviz", "se3_sensor.rviz"]
    )

    # We setup the controller manager node 
    control_node = Node(
        package="controller_manager", 
        executable="ros2_control_node", 
        parameters=[robot_description, # we need to pass the urdf (parameters applied) and config (yaml file)
                    se3_controllers,
                    {"robot_sensor_name": robot_sensor_name},
                    {"robot_frame_id": robot_frame_id},
                    {"target_sensor_name": target_sensor_name},
                    {"target_frame_id": target_frame_id}], 
        output="both"
    )

    # Start the robot description for visualization via the robot state publisher 
    robot_state_pub_node = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        parameters=[robot_description], # As we have seen from the ros2 doc, the robot state publisher needs the urdf
        output="both"
    )

    # RViz node 
    rviz_node = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="log", 
        arguments=["-d", rviz_config_file]
    )

    # Might need to add Joint State Broadcaster at some point 

    # We spawn the robot sensor's pose broadcaster 
    robot_pose_broadcaster_spawner = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["robot_pose_broadcaster", # The name of the controller to spawn 
                    "--controller-manager",  # Some kinda flag used to specify the next argument
                    "/controller_manager"] # The node name  
    )

    # We spawn the target sensor's pose broadcaster
    target_pose_broadcaster_spawner = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["target_pose_broadcaster", "--controller-manager", "/controller_manager"]
    )

    # TF Lookup Server Node
    tf_lookup_server_node = Node(
        package="se3_sensor_driver", 
        executable="tf_lookup_server",
        name="tf_lookup_server",
        output="screen"
    )

    # Nodes to Start 
    nodes_to_start = [
        tf_lookup_server_node,
        control_node, 
        robot_state_pub_node,  
        robot_pose_broadcaster_spawner,
        target_pose_broadcaster_spawner, 
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)