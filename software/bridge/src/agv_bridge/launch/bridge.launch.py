from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='UART device file the ESP32-C3 enumerates as. The 99-agv.rules '
                    'udev file creates this symlink; without it use '
                    '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit-if00.',
    )
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='921600',
        description='UART baud rate. Must match firmware/config.h.',
    )
    ws_host_arg = DeclareLaunchArgument(
        'ws_host', default_value='0.0.0.0',
        description='Interface the WebSocket server binds to.',
    )
    ws_port_arg = DeclareLaunchArgument(
        'ws_port', default_value='8765',
        description='Port for the workstation GUI to connect to.',
    )
    ws_path_arg = DeclareLaunchArgument(
        'ws_path', default_value='/ws',
        description='WebSocket path. GUI default expects /ws.',
    )
    uno_port_arg = DeclareLaunchArgument(
        'uno_port', default_value='/dev/ttyUSB0',
        description='Serial device for the Arduino UNO status panel. The '
                    '99-agv.rules udev file creates this symlink.',
    )
    panel_rate_arg = DeclareLaunchArgument(
        'panel_rate_hz', default_value='5.0',
        description='Status-panel refresh rate sent to the Arduino.',
    )

    uart_node = Node(
        package='agv_bridge',
        executable='uart_bridge_node',
        name='uart_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud': LaunchConfiguration('baud'),
        }],
    )

    ws_node = Node(
        package='agv_bridge',
        executable='ws_bridge_node',
        name='ws_bridge',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('ws_host'),
            'port': LaunchConfiguration('ws_port'),
            'path': LaunchConfiguration('ws_path'),
        }],
    )

    panel_node = Node(
        package='agv_bridge',
        executable='panel_node',
        name='panel_node',
        output='screen',
        parameters=[{
            'uno_port': LaunchConfiguration('uno_port'),
            'rate_hz': LaunchConfiguration('panel_rate_hz'),
        }],
    )

    return LaunchDescription([
        serial_port_arg, baud_arg,
        ws_host_arg, ws_port_arg, ws_path_arg,
        uno_port_arg, panel_rate_arg,
        uart_node, ws_node, panel_node,
    ])
