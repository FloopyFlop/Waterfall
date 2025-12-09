from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mav_inject'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 node for editing PX4 drone configuration files during flight operations',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'injection_test = mav_inject.injection_test:main',
            'config_controller = mav_inject.config_controller:main',
            'param_lister = mav_inject.param_lister:main',
        ],
    },
)
