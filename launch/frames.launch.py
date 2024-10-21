from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)


def generate_launch_description():


    declared_arguments = []


    declared_arguments.append(
        DeclareLaunchArgument(
            "ang_vel", #this will be the name of the argument  
            default_value="0.2",
            description="angular vel omega_z for the rotating frame",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "lin_vel", #this will be the name of the argument  
            default_value="0.1",
            description="anglinear vel along z the rotating_frame",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", #this will be the name of the argument  
            default_value=PathJoinSubstitution(
                [FindPackageShare("interview_homework"), "config", "rviz", "configuration.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )


    frames = Node(
        package="interview_homework",
        executable="pb_frames",
        name='pb_frames_node',
        parameters=[{'angvel': LaunchConfiguration("ang_vel")},
                    {'linvel': LaunchConfiguration("lin_vel")}
                    ]
    )  

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    im_pub = Node(
        package="interview_homework",
        executable="im_pub",
        name='im_pub', 
    )

  
    nodes_to_start = [
        frames,
        rviz_node,
        im_pub
    ]
    
    return LaunchDescription(declared_arguments + nodes_to_start) 

