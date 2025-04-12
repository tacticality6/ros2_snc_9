from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'ros2_snc_9'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmitaiil',
    maintainer_email='s39477643@student.rmit.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_node_executable = ros2_snc_9.robot_state_server:main',
            'nav_node_executable = ros2_snc_9.navigation_logic_node:main',
            'hazard_detection_executable = ros2_snc_9.marker_detection_node:main'
        ],
    },
)
