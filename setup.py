from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'waterfall'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=[
        'setuptools',
        'pymavlink>=2.4.40',
        'rich>=13.0.0',
        'mavsdk>=1.9.0',
        'numpy',
        'matplotlib',
        'scipy',
    ],
    package_data={
        'waterfall': ['px4_param_bridge.c'],
    },
    zip_safe=True,
    maintainer='waterfall',
    maintainer_email='dev@todo.todo',
    description='ROS2 manager for Orchestra, Firehose, UniformPump, and Inject.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waterfall_node = waterfall.manager:main',
            'firehose_service = waterfall.firehose_service:main',
            'uniform_pump_service = waterfall.uniform_pump_service:main',
            'inject_service = waterfall.inject_service:main',
            'orchestra_service = waterfall.orchestra_service:main',
            'waterfall_coordinator = waterfall.coordinator:main',
            'waterfall_fault_demo = waterfall.fault_monitor_demo:main',
            'waterfall_motor_finder = waterfall.motor_reverse_finder:main',
        ],
    },
)
