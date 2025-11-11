import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the Argo simulation environment.

    This launch file starts the essential nodes for running the simulation,
    including the simulator bridge, the controller, and visualization tools.
    It uses ExecuteProcess to run nodes as standalone Python scripts, which is
    suitable for projects that are not built as formal ROS2 packages.
    """

    # Get the root directory of the Argo project. The launch file is in
    # ARGO_DIR/launch, so we navigate up one level.
    argo_dir = os.path.join(os.path.dirname(__file__), '..')

    # Define the full paths to the Python scripts for each node
    bridge_script = os.path.join(argo_dir, 'nodes', 'argo_unified_simulator_bridge.py')
    controller_script = os.path.join(argo_dir, 'nodes', 'controller.py')
    sailing_area_script = os.path.join(argo_dir, 'nodes', 'sailing_area_publisher.py')

    # --- Node: argo_unified_simulator_bridge ---
    # This node provides the core simulation logic.
    simulator_bridge_node = ExecuteProcess(
        cmd=['python3', bridge_script, '--mode', 'local', '--no-curses'],
        name='argo_unified_simulator_bridge',
        output='screen'
    )

    # --- Node: controller ---
    # The controller node is responsible for the boat's autonomous behavior.
    controller_node = ExecuteProcess(
        cmd=['python3', controller_script],
        name='controller_node',
        output='screen'
    )

    # --- Node: sailing_area_publisher ---
    # Publishes sailing area boundaries for visualization in Foxglove.
    sailing_area_publisher_node = ExecuteProcess(
        cmd=['python3', sailing_area_script],
        name='sailing_area_publisher',
        output='screen'
    )

    # --- Node: foxglove_bridge ---
    # This provides the websocket server for Foxglove Studio to connect to.
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        simulator_bridge_node,
        controller_node,
        sailing_area_publisher_node,
        foxglove_bridge_node,
    ])
