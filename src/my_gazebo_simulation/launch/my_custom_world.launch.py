import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    my_gazebo_simulation_pkg = get_package_share_directory('my_gazebo_simulation')
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    turtlebot_model = LaunchConfiguration('turtlebot_model', default='burger')
    
    #world_file = os.path.join(my_gazebo_simulation_pkg, 'worlds', 'custom_world.world')
    world_file = os.path.join(my_gazebo_simulation_pkg, 'worlds', 'SimpleENV.world')
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
            
        ),
        launch_arguments= {'world': world_file}.items()
    )
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'model': turtlebot_model}.items()
    )
    
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'model' : turtlebot_model
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('turtlebot_model', default_value='burger'))
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    
    return ld