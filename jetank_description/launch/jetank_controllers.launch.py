import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration , PythonExpression

def generate_launch_description():


    jetank_package = get_package_share_directory('jetank_description')
    jetank_controllers_config = os.path.join(jetank_package,"config","jetank_controllers.yaml")

    robot_namespace = LaunchConfiguration('ns')
    robot_namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='[ARG] required namespace to keep nodes and topics separate when running multiple robots in the same simulation'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='[ARG] tells the robot_state_publisher to use the simulation time or just unix timestamp'
    )

    # define the controller nodes that communicate
    # between the DDS topics and the ros2_control control manager
    # NOTE: controllers can and must not claim the same command_interfaces
    # you can find this in your YAML file that sets the configuration for these
    # controllers and which joints they have claim on
    # to see why i add the "--switch-timeout" argument: https://github.com/ros-controls/ros2_control/pull/1790
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=[
            'joint_state_broadcaster',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10',
            ]
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=[
            'diff_drive_controller',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10',
            ],
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=[
            'arm_controller',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10',
            ]
     )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=[
            'gripper_controller',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10',
        ]
    )

    # Start the controller manager
    # required to handle the controllers nodes I/O
    # and ros2_control resource manager
    # NOTE: due to gz_ros2_control it is not required to run this 
    # code but it could be that outside a simulation this might be required
    # https://github.com/ros-controls/gz_ros2_control
    # https://github.com/ros-controls/gz_ros2_control/blob/rolling/doc/index.rst
     
    # 1) primary option outside the simulation
    # controller_manager_node = Node(
    #     output="screen",
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     namespace=robot_namespace,
    #     parameters=[
    #         jetank_controllers_config,
    #     ],
    #     remappings=[
    #         ("~/robot_description", "robot_description"),
    #     ]
    # )

    # 2) remap inside the URDF files
    

    # event handlers that will ensure that some nodes
    # are started in a lineair fashion:
    # in this case we assign a event handler on the joint_state_broadcaster
    # once the process stops it launches the controller nodes
    joint_state_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diff_drive_controller_spawner,
                arm_controller_spawner,
                gripper_controller_spawner
            ]
    ))     
   
    return LaunchDescription([
        # required arguments
        robot_namespace_arg,
        use_sim_time_arg,

        # NOTE: this can be ignored but outside the simulation 
        # it might be neccessary
        # controller_manager_node,

        
        # required eventhandler and controller node
        joint_state_broadcaster_spawner,
        joint_state_event_handler,
    ])
