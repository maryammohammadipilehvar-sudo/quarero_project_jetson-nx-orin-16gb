from setuptools import setup

package_name = 'video_ringbuffer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Video ringbuffer node for security event recording',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ringbuffer_node = video_ringbuffer.ringbuffer_node:main',
        ],
    },
)


