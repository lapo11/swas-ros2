from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'swas_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'calib'), glob('calib/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lapo11',
    maintainer_email='lapo.veca@gmail.com',
    description='ROS 2 nodes for the SWAS battery swapping system (BXU + BRU)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bxu_motion_node = swas_system.bxu.motion_node:main',
            'bxu_front_vision_node = swas_system.bxu.front_vision_node:main',
            'bxu_front_approach_controller_node = swas_system.bxu.front_approach_controller_node:main',
            'bxu_docking_vision_node = swas_system.bxu.docking_vision_node:main',
            'bxu_docking_controller_node = swas_system.bxu.docking_controller_node:main',
            'bxu_supervisor_node = swas_system.bxu.supervisor_node:main',
            'bxu_battery_lift_node = swas_system.bxu.battery_lift_node:main',
        ],
    },
)
