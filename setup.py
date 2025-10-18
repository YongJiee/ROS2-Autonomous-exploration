from setuptools import setup
import os
from glob import glob

package_name = 'maze_explorer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Include rviz files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # Include model files
        (os.path.join('share', package_name, 'models', 'Maze_01'),
            glob('models/Maze_01/*')),
        (os.path.join('share', package_name, 'models', 'Maze02'),
            glob('models/Maze02/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yongjie',
    maintainer_email='your@email.com',
    description='Maze exploration package for TurtleBot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer = maze_explorer.explorer_node:main',
            'nav_goal_sender = maze_explorer.nav_goal_sender:main',
        ],
    },
)