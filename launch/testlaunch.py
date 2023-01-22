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
                name='node_container1',
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
                name='node_container2',
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
                name='node_container3',
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
        turtle_reset = ComposableNodeContainer(
                name='node_container4',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                        ComposableNode(
                                package='software_training',
                                plugin='turtle_composition::teleport_turtle',
                                name='teleport_turtle',
                        )
                ]
        )


        service_name = " /reset_moving_turtle"
        service_location = " training_interfaces/srv/TurtleReset"
        call_service = ExecuteProcess(
                cmd=[[
                FindExecutable(name='ros2'),
                ' service call',
                service_name,
                service_location
                ]],
                shell=True
        )


        #container for publisher
        turtle_distance = ComposableNodeContainer(
                name='node_container5',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                        ComposableNode(
                                package='software_training',
                                plugin='turtle_composition::turtle_distance_publisher',
                                name='turtle_distance',
                        )
                ]
        )

        # action stuff
        action_server = ComposableNodeContainer(
                name='node_container6',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                        ComposableNode(
                                package='software_training',
                                plugin='turtle_composition::turtle_action_server',
                                name='turtle_action_server',
                        )
                ]
        )
        action_location = " training_interfaces/action/Software"
        action_name = " /software_action_"
        call_action = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' action send_goal ',
            action_name,
            action_location,
            ' " {x: 7 , y: 7}"'
        ]],
        shell=True
    )



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
                period=7.0,
                actions=[turtle_clear]
        )
        
        
        #runs the make circle node after the turtlesim node is made
        gs = RegisterEventHandler(
                event_handler=OnProcessStart(
                        target_action=launch_turtlesim,
                        on_start=[turtle_move_circle,turtle_spawn],
                )
        )
        
        #calls clear after turtle makes circle
        ds = RegisterEventHandler(
                event_handler=OnExecutionComplete(
                        target_action=turtle_move_circle,
                        on_completion=[turtle_spawn],
                )
        )

        #call spawn after clear THIS IS THE BROKEN ONE
        ts = RegisterEventHandler(
                event_handler=OnExecutionComplete(
                        target_action=turtle_spawn,
                        on_completion=[timer_test],
                )
        )

        #reset
        ys = TimerAction(period=3.0,actions=[call_service])
        

        ms = RegisterEventHandler(
                event_handler=OnExecutionComplete(
                        target_action=turtle_move_circle,
                        on_completion=[turtle_distance,turtle_reset,ys,TimerAction(period=4.0,actions=[action_server]),TimerAction(period=5.0,actions=[call_action]),timer_test])
        )

        
        #-------------------------------------------------------------------------------
        return launch.LaunchDescription([launch_turtlesim,gs,ds,ms])
        # return launch.LaunchDescription([launch_turtlesim,gs,ds,ts])
