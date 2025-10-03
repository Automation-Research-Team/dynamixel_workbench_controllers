from launch                   import LaunchDescription
from launch.actions           import (DeclareLaunchArgument, OpaqueFunction,
                                      GroupAction)
from launch.substitutions     import (LaunchConfiguration,
                                      PathJoinSubstitution, EqualsSubstitution)
from launch.conditions        import IfCondition, UnlessCondition
from launch_ros.actions       import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions  import ComposableNode


launch_arguments = [
    {
        'name':        'name',
        'default':     'dynamixel_workbench',
        'description': 'name of the controller'
    },
    {
        'name':        'config_file',
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
        'default':     '',
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

parameter_arguments = [
    {
        'name':        'dynamixel_info',
        'default':     PathJoinSubstitution([
                           FindPackageShare('dynamixel_workbench_controllers'),
                           'config', 'basic.yaml']),
        'description': 'path to YAML file for configuring dynamixel motors'
    }
]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def set_configurable_parameters(args):
    return {arg['name']: LaunchConfiguration(arg['name']) for arg in args}

def launch_setup(context, param_args):
    config_file   = LaunchConfiguration('config_file')
    config_params = set_configurable_parameters(param_args)
    return [
        Node(name=LaunchConfiguration('name'),
             package='dynamixel_workbench_controllers',
             executable='dynamixel_workbench_controllers_node',
             parameters=[config_file, config_params],
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')],
             emulate_tty=True,
             condition=IfCondition(
                 EqualsSubstitution(LaunchConfiguration('container'), ''))),
        GroupAction(
            condition=UnlessCondition(
                EqualsSubstitution(LaunchConfiguration('container'), '')),
            actions=[
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
                            parameters=[config_file, config_params],
                            extra_arguments=[
                                {'use_intra_process_comms': True}])])])
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments +
                                                      parameter_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
