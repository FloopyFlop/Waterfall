from setuptools import setup
import os
from glob import glob

package_name = 'mav_data_hose'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'pymavlink>=2.4.40',
        'rich>=13.0.0',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='MAV FIRE HOSE - High-frequency raw MAVLink data streaming to ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'firehose_node = mav_data_hose.firehose_node:main',
            'uniform_pump_node = mav_data_hose.uniform_pump_node:main',
        ],
    },
)
