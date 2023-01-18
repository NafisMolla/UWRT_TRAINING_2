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

        start_turtlesim = Node(
                package="turtlesim",
                executable="turtlesim_node",
                name="turtlesim1"
        )

        #-------------------------------container for turtle_clear-----------------------------------
        
        turtle_clear = ComposableNodeContainer(
                name='node_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                        ComposableNode(
                                package='software_training',
                                plugin='kill_all_composition::kill_all_turtle_service_call',
                                name='kill_all_turtles',
                        )                      

                ]
        )
        

        #------------------------------container for move turtle in circle -------------------------------------


        #container for turtlespawn
        turtle_move_circle = ComposableNodeContainer(
                name='node_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                        ComposableNode(
                                package='software_training',
                                plugin='circular_move_namespace::turtle_move_circular',
                                name='move_turtle',
                        )
                ]
        )

        #container for turtle spawn service call
        turtle_spawn = ComposableNodeContainer(
                name='node_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                        ComposableNode(
                                package='software_training',
                                plugin='spawn_composition::spawn_turtle',
                                name='spawn_turtle',
                        )
                ]
        )


        #container for turtle move service call


        #container for publisher


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
        #         ),
            ComposableNode(
                package='software_training',
           plugin='circular_move_namespace::turtle_move_circular',
                name='move_turtle',
                )
        #     ComposableNode(
        #             package='software_training',
        #        plugin='spawn_composition::spawn_turtle',
        #             name='spawn_turtle',
        #             ),
        # ComposableNode(
        #         package='software_training',
        # plugin='turtle_composition::teleport_turtle',
        #         name='teleport_turtle',
        #         )
        ],
        output='screen',
        )

        #-----------------------------------------------------------------------------
        
        
        launch_turtlesim = launch_ros.actions.Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='turtlesim',
                output='screen'
        )

        timer_test = launch.actions.TimerAction(
                period=10.0,
                actions=[turtle_spawn]
        )
        
        
        #runs the make circle node after the turtlesim node is made
        gs = RegisterEventHandler(
                event_handler=OnProcessStart(
                        target_action=launch_turtlesim,
                        on_start=[turtle_move_circle],
                )
        )
        
        #calls clear after turtle makes circle
        ds = RegisterEventHandler(
                event_handler=OnExecutionComplete(
                        target_action=turtle_move_circle,
                        on_completion=[turtle_clear],
                )
        )

        #call spawn after clear

        ts = RegisterEventHandler(
                event_handler=OnExecutionComplete(
                        target_action=turtle_clear,
                        on_completion=[turtle_spawn],
                )
        )
        #-------------------------------------------------------------------------------


        return launch.LaunchDescription([launch_turtlesim,gs,ds,ts])
        # return launch.LaunchDescription([ds,test1])
