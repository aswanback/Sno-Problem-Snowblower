from setuptools import setup
import os
from glob import glob

package_name = 'sno'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}/lib'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('fake_nodes/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compass_node = sno.compass_node:main',
            'control_node = sno.control_node:main',
            'flutter_node = sno.flutter_node:main',            
            'gps_node = sno.gps_node:main',
            'handle_node = sno.handle_node:main',
            'mode_node = sno.mode_node:main',
            'motor_node = sno.motor_node:main',
            'navigation_node = sno.navigation_node:main',
            'stepper_node = sno.stepper_node:main',
            'ultrasonic_node = sno.ultrasonic_node:main',
            'waypoint_node = sno.waypoint_node:main',

            'fake_control_node = sno.fake_nodes.fake_control_node:main',
            'fake_zone_node = sno.fake_nodes.fake_zone_node:main',
            # 'fake_location_heading_node = sno.fake_nodes.fake_location_heading_node:main',
        ],
    },
)
