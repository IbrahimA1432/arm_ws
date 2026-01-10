import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os
package_name='robotic_arm_6dof'
urdf_path = 'urdf/robotic_arm_6dof.urdf'

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
    urdfModelPath= os.path.join(pkgPath, urdf_path)
    
    with open(urdfModelPath,'r') as infp:
        robot_desc = infp.read()
    params = {'robot_description': robot_desc}
    
    robot_state_publisher_node =launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[params])
    
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        #arguments=['-d' + os.path.join(get_package_share_directory(package_name), 'config', 'config.rviz')]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='This is a flag for joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdfModelPath,
                                            description='Path to the urdf model file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]) 