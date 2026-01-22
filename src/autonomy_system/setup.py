import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autonomy_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominik Wojtasik',
    maintainer_email='dominikwojtasik191@gmail.com',
    description="""This package provides a behavior tree–based mission controller for the RoboSub competition. 
    It defines task sequences and execution nodes responsible for autonomous mission management.""",
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'behavior_tree = autonomy_system.behavior_tree_node:main',
            'hello_world_action = autonomy_system.behaviours.HelloWorld:main',
            'stabilize_on_position_action = autonomy_system.behaviours.StabilizeOnPosition:main',
            'dive_task_action = autonomy_system.behaviours.DiveTask:main',
            'move_to_gate_action = autonomy_system.behaviours.MoveToGate:main',
            'search_gate_action = autonomy_system.behaviours.SearchGate:main',
        ],
    },
)
