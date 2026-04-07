from setuptools import find_packages, setup

package_name = 'robot_web_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 FastAPI web interface for the robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_web_interface = robot_web_interface.main:main',
            'test_script = robot_web_interface.test:main',
        ],
    },
)