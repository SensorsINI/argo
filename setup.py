from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'argo_power_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Argo Team',
    maintainer_email='argo@example.com',
    description='Argo Power Control System - ROS2 node for power management and LED control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'argo_power_control = argo_power_control.argo_power_control:main',
        ],
    },
)