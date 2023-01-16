import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)



def generate_launch_description():


    continuous_node_container = ComposableNodeContainer(
            name='node_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
            #     ComposableNode(
            #         package='software_training',
            #    plugin='kill_all_composition::kill_all_turtle_service_call',
            #         name='kill_all_turtles',
            #         )
                ComposableNode(
                    package='software_training',
               plugin='circular_move_namespace::turtle_move_circular',
                    name='move_turtle',
                    )
            ],
            output='screen',
    )


    return launch.LaunchDescription([continuous_node_container])