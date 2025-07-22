import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Path and Argument Declarations ---

    # Declare the 'use_sim_time' argument, which is crucial for simulation
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    # Get the value of use_sim_time to pass to nodes
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Define paths to important files and directories
    
    pkg_share_dir = get_package_share_directory('carto')
    urdf_path = os.path.join(pkg_share_dir, 'config', 'transformation.urdf.xacro')
    lua_config_dir = os.path.join(pkg_share_dir, 'config')
    map_file_path = os.path.join(pkg_share_dir, 'map', 'my_map.pbstream')
    rviz_config_path = os.path.join(pkg_share_dir, 'rviz', 'rviz_config.rviz') 
    # Declare the argument for the map file path
    load_state_filename_arg = DeclareLaunchArgument(
        'load_state_filename',
        default_value=map_file_path,
        description='Path to the .pbstream map file to load for localization'
    )
    # Get the value of the map file path
    load_state_filename = LaunchConfiguration('load_state_filename')

    # Process the URDF file
    robot_description_content = xacro.process_file(urdf_path).toxml()

    # --- Node Definitions ---

    # Robot State Publisher: Publishes TF transforms from the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Cartographer Node: Runs the SLAM algorithm in localization mode
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', lua_config_dir,
            '-configuration_basename', 'localisation.lua',
            '-load_state_filename', load_state_filename,
        ],
        remappings=[
            ('points2', '/velodyne_points'),
            ('imu', '/imu/data') 
        ]
    )

    # Occupancy Grid Node: Creates a 2D costmap from the 3D map for navigation
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}
        ]
    )

    # RViz Node: Visualizes the robot model, map, and sensor data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        on_exit=Shutdown(),
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- LaunchDescription Return ---
    
    return LaunchDescription([
        # Add launch arguments to the LaunchDescription
        use_sim_time_arg,
        load_state_filename_arg,

        # Add nodes to the LaunchDescription
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        # rviz_node
    ])
