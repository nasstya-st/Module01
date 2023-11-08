import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import launch_ros

#sdf format urdf

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    #pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    #pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    #pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')

    # Load the SDF file from "description" package
    pkg_share = launch_ros.substitutions.FindPackageShare(package='circle_movement').find('circle_movement')
    
    urdf_file = os.path.join(pkg_share,'description', 'fox_lidar.urdf.xacro')
    
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
        	os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
    	),
    	launch_arguments={
        	"gz_args": f"-r empty.sdf"
    	}.items(),
	)

    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'fox', '-x 0.6', '-y 0.0', '-z 0.18',
                   '-topic', 'robot_description'],
        output='screen'          
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro ', urdf_file])},
            {'frame_prefix' : 'fox/'},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_share, 'config', 'lidar.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    #gz_topic = '/model/fox'
    #link_pose_gz_topic = gz_topic + '/pose'
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'fox_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    lidar_avoidance = Node(
        package='circle_movement',
        executable='lidar_avoidance',
        name='lidar_avoidance',
        parameters=[
        ]
    )

    


    return LaunchDescription([
        gz_sim,
        create,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        robot_state_publisher,
        rviz,
        #lidar_avoidance,
    ])
