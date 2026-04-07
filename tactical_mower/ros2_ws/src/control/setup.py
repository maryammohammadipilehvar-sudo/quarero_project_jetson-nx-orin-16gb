from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control'

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
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='info@ai-robotic.de',
    description='Robot control package providing navigation, obstacle avoidance, mission scheduling, and state machine management for autonomous operation.',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_controller = control.joy_controller_node:main',
            'tactical_wp_follower = control.nodes.tactical_wp_follower_node:main',
            'person_detection_bridge = control.nodes.person_detection_bridge:main',
            'tactical_scheduler = control.nodes.tactical_scheduler_node:main',
            'sector_publisher = control.obstacle_avoidance.sector_analysis.sector_publisher_node:main',
            'sector_viz = control.obstacle_avoidance.sector_analysis.sector_viz_node:main',
            'obstacle_viz = control.obstacle_avoidance.obstacle_viz_node:main',
            'fusion_node = control.obstacle_avoidance.fusion_node:main',
        ],
    },
)