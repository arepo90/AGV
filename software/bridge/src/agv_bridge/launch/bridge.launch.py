from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ESP',
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
        'uno_port', default_value='/dev/UNO',
        description='Serial device for the Arduino UNO status panel. The '
                    '99-agv.rules udev file creates this symlink.',
    )
    panel_rate_arg = DeclareLaunchArgument(
        'panel_rate_hz', default_value='5.0',
        description='Status-panel refresh rate sent to the Arduino.',
    )
    lidar_fov_min_arg = DeclareLaunchArgument(
        'lidar_fov_min_deg', default_value='80.0',
        description='LiDAR usable FOV start, normalised to [0, 360) '
                    '(maps to the "0°" LED indicator point).',
    )
    lidar_fov_max_arg = DeclareLaunchArgument(
        'lidar_fov_max_deg', default_value='280.0',
        description='LiDAR usable FOV end, normalised to [0, 360); 90-270 covers the '
                    'rear half (wraps through +-180°) (maps to the "MAX_FOV°" LED indicator point).',
    )
    lidar_bin_arg = DeclareLaunchArgument(
        'lidar_bin_deg', default_value='15.0',
        description='LiDAR angular bin width Z° (average distance per interval).',
    )
    lidar_mask_min_arg = DeclareLaunchArgument(
        'lidar_mask_min_deg', default_value='1.0',
        description='LiDAR occluded-sector start X° (mask disabled while min > max).',
    )
    lidar_mask_max_arg = DeclareLaunchArgument(
        'lidar_mask_max_deg', default_value='-1.0',
        description='LiDAR occluded-sector end Y°.',
    )   
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar', default_value='true',
        description='Start lidar_node. Off by default — the LiDAR is not connected yet. '
                    'Re-enable with enable_lidar:=true (and set DISABLE_LIDAR=0 in the STM32 '
                    'config.h so the firmware acts on the segments).',
    )
    song_path_arg = DeclareLaunchArgument(
        'song_path', default_value='/home/arepo/AGV/software/bridge/src/agv_bridge/resource/song.mp3',
        description='Audio file played by jukebox_node while REMOTE_CONTROL motion '
                    'commands are flowing (any SDL_mixer-decodable format).',
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

    lidar_node = Node(
        package='agv_bridge',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        parameters=[{
            'fov_min_deg': LaunchConfiguration('lidar_fov_min_deg'),
            'fov_max_deg': LaunchConfiguration('lidar_fov_max_deg'),
            'bin_deg': LaunchConfiguration('lidar_bin_deg'),
            'mask_min_deg': LaunchConfiguration('lidar_mask_min_deg'),
            'mask_max_deg': LaunchConfiguration('lidar_mask_max_deg'),
        }],
    )

    jukebox_node = Node(
        package='agv_bridge',
        executable='jukebox_node',
        name='jukebox_node',
        output='screen',
        parameters=[{
            'song_path': LaunchConfiguration('song_path'),
        }],
    )

    return LaunchDescription([
        serial_port_arg, baud_arg,
        ws_host_arg, ws_port_arg, ws_path_arg,
        uno_port_arg, panel_rate_arg,
        lidar_fov_min_arg, lidar_fov_max_arg, lidar_bin_arg,
        lidar_mask_min_arg, lidar_mask_max_arg, enable_lidar_arg,
        song_path_arg,
        uart_node, ws_node, panel_node, lidar_node, jukebox_node,
    ])
