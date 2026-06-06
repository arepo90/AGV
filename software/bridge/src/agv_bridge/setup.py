from setuptools import setup
from glob import glob
import os

package_name = 'agv_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV',
    maintainer_email='nabetse069@gmail.com',
    description='UART and WebSocket bridges between firmware and workstation GUI.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'uart_bridge_node = agv_bridge.uart_bridge_node:main',
            'ws_bridge_node = agv_bridge.ws_bridge_node:main',
            'panel_node = agv_bridge.panel_node:main',
            'lidar_node = agv_bridge.lidar_node:main',
        ],
    },
)
