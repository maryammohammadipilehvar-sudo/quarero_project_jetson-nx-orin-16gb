from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='info@ai-robotic.de',
    description='Robot drive package providing joystick control, kinematics calculations, motor control, and hardware abstraction for differential drive platform.',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_controller = drive.nodes.robot_controller_node:main',
            'roboclaw_wrapper = drive.nodes.roboclaw_wrapper_node:main',
        ],
    },
)
