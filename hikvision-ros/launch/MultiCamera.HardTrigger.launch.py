from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """launch内容描述函数, 由ros2 launch扫描调用"""
    hikvision_ros_pkg_dir = get_package_share_directory('hikvision-ros')
    default_params_file = \
        Path(hikvision_ros_pkg_dir) / 'config' / 'multi_camera_params.yaml'
    
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')
    
    hikvision_ns = LaunchConfiguration('hikvision_ns')
    hikvision_ns_arg = DeclareLaunchArgument(
        'hikvision_ns', default_value='hikvision')
    
    camera_1 = Node(
        package='hikvision-ros',
        executable='ImgHardTriggerNode',
        name='camera_1',
        namespace=hikvision_ns,
        parameters=[params_file],
        output='screen',
    )

    camera_2 = Node(
        package='hikvision-ros',
        executable='ImgHardTriggerNode',
        name='camera_2',
        namespace=hikvision_ns,
        parameters=[params_file],
        output='screen',
    )

    camera_3 = Node(
        package='hikvision-ros',
        executable='ImgHardTriggerNode',
        name='camera_3',
        namespace=hikvision_ns,
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        hikvision_ns_arg,
        camera_1,
        camera_2,
        camera_3
    ])