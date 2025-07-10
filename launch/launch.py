import yaml
from launch                   import LaunchDescription
from launch.actions           import (DeclareLaunchArgument, OpaqueFunction,
                                      GroupAction)
from launch.substitutions     import (LaunchConfiguration,
                                      PathJoinSubstitution, EqualsSubstitution)
from launch.conditions        import IfCondition, UnlessCondition
from launch_ros.actions       import Node, LoadComposableNodes
from launch_ros.descriptions  import ComposableNode

launch_arguments = [
    {'name':        'namespace',
     'default':     '',
     'description': 'namespace for the controller'},
    {'name':        'name',
     'default':     'dynamixel_workbench',
     'description': 'name of the controller'},
    {'name':        'config_file',
     'default':     '',
     'description': 'path to YAML file for configuring the controller'},
    {'name':        'external_container',
     'default':     'false',
     'description': 'use existing external container'},
    {'name':        'container',
     'default':     '',
     'description': 'name of internal or external component container'},
    {'name':        'log_level',
     'default':     'info',
     'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
    {'name':        'output',
     'default':     'screen',
     'description': 'pipe node output [screen|log|both]'}]

parameter_arguments = [
    {'name':        'usb_port',
     'default':     '/dev/ttyUSB0',
     'description': 'device file name of the USB port'},
    {'name':        'dxl_baud_rate',
     'default':     '57600',
     'description': 'baud rate of the Dynamixel'},
    {'name':        'use_joint_state_topic',
     'default':     'false',
     'description': 'publish joint state'},
    {'name':        'use_moveit',
     'default':     'false',
     'description': 'pass through joint trajectory computed by MoveIt'}]

def declare_launch_arguments(args, defaults={}):
    num_to_str = lambda x : str(x) if isinstance(x, (bool, int, float)) else x
    return [DeclareLaunchArgument(
                arg['name'],
                default_value=num_to_str(defaults.get(arg['name'],
                                                      arg['default'])),
                description=arg['description']) \
            for arg in args]

def load_parameters(config_file):
    if config_file == '':
        return {}
    with open(config_file, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def set_configurable_parameters(args):
    return dict([(arg['name'], LaunchConfiguration(arg['name'])) \
                 for arg in args])

def launch_setup(context, param_args):
    params   = load_parameters(
                   LaunchConfiguration('config_file').perform(context))
    actions  = declare_launch_arguments(param_args, params)
    params  |= set_configurable_parameters(param_args)
    actions += [Node(namespace=LaunchConfiguration('namespace'),
                     name=LaunchConfiguration('name'),
                     package='dynamixel_workbench_controllers',
                     executable='dynamixel_workbench_controllers_node',
                     parameters=[params],
                     output=LaunchConfiguration('output'),
                     arguments=['--ros-args', '--log-level',
                                LaunchConfiguration('log_level')],
                     emulate_tty=True,
                     condition=IfCondition(
                                   EqualsSubstitution(
                                       LaunchConfiguration('container'), ''))),
                GroupAction(
                    condition=UnlessCondition(
                                  EqualsSubstitution(
                                      LaunchConfiguration('container'), '')),
                    actions=[
                        Node(name=LaunchConfiguration('container'),
                             package='rclcpp_components',
                             executable='component_container',
                             output=LaunchConfiguration('output'),
                             arguments=['--ros-args', '--log-level',
                                        LaunchConfiguration('log_level')],
                             condition=UnlessCondition(
                                 LaunchConfiguration('external_container'))),
                        LoadComposableNodes(
                            target_container=LaunchConfiguration('container'),
                            composable_node_descriptions=[
                                ComposableNode(
                                    namespace=LaunchConfiguration('namespace'),
                                    name=LaunchConfiguration('name'),
                                    package='dynamixel_workbench_controllers',
                                    plugin='dynamixel_workbench_controllers::DynamixelController',
                                    parameters=[params],
                                    extra_arguments=[
                                        {'use_intra_process_comms': True}])])])
                ]
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
