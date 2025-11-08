from launch                   import LaunchDescription
from launch.actions           import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions     import (LaunchConfiguration,
                                      PathJoinSubstitution)
from launch.conditions        import UnlessCondition
from launch_ros.actions       import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions  import ComposableNode


launch_arguments = [
    {
        'name':        'name',
        'default':     'basic_driver',
        'description': 'name of the controller'
    },
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('dynamixel_workbench_controllers'),
                           'config', 'default.yaml']),
        'description': 'path to YAML file for configuring the controller'
    },
    {
        'name':        'external_container',
        'default':     'false',
        'description': 'use existing external container',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'container',
        'default':     'dynamixel_workbench_container',
        'description': 'name of internal or external component container'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'screen',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def launch_setup(context):
    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')],
             condition=UnlessCondition(
                 LaunchConfiguration('external_container'))),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    name=LaunchConfiguration('name'),
                    package='dynamixel_workbench_controllers',
                    plugin='dynamixel_workbench_controllers::DynamixelController',
                    parameters=[LaunchConfiguration('param_file')],
                    extra_arguments=[{'use_intra_process_comms': True}])])
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
