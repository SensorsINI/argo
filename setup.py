"""
ROS2 Package Setup for Argo Sailboat

NOTE: This setup.py is currently NOT used in the project's deployment.

CURRENT DEPLOYMENT METHOD:
- Scripts are executed directly from the project's scripts/ directory
- Launch uses relative paths, not installed packages
- Managed via systemd services (see Makefile and launch/*.service files)
- No colcon build process required

WHEN THIS SETUP.PY WOULD BE NEEDED:
- If you want to install as a proper ROS2 package: pip install -e .
- If you want to use: ros2 run argo <script_name>
- If transitioning to colcon build workflow
- If distributing the package to others

TO USE THIS SETUP.PY:
1. Run: pip install -e . (from project root)
2. Or: colcon build --packages-select argo
3. Then: ros2 run argo <script_name>

CURRENT STATUS: Optional/Unused - project works without it
"""

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'argo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.ini')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        # Install scripts as executables
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tobi Delbruck',
    maintainer_email='tobi@ini.uzh.ch',
    description='The argo robot sailboat package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add entry points for your main scripts if needed
            # 'anem = argo.scripts.anem:main',
            # 'gps = argo.scripts.gps:main',
            # etc.
        ],
    },
)
