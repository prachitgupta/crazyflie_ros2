from setuptools import setup
import os
from glob import glob

package_name = 'experiments'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Crazyflie experiments',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = experiments.keyboard_teleop:main',
            'takeoff_node = experiments.takeoff_node:main',
            'land_node = experiments.land_node:main',
            'circle_flight_node = experiments.circle_flight_node:main',
            'hover_above_jackal = experiments.hover_above_jackal:main',
            'circular_tracking_node = experiments.circular_tracking_node:main',
        ],
    },
)