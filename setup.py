import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'freenove_4wd_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tim',
    maintainer_email='tim@todo.todo',
    description='ROS driver for Freenove 4wd platform',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = freenove_4wd_ros.driver:main',
            'test_motors = freenove_4wd_ros.freenove4wd:test_motors',
            'test_full_speed_straight = freenove_4wd_ros.freenove4wd:test_full_speed_straight',
        ],
    },
)
