from setuptools import find_packages, setup

package_name = 'swas_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
