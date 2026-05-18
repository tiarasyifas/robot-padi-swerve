from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swerve_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Launch files (jika ada)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Padi Team',
    maintainer_email='your@email.com',
    description='Swerve Drive Controller ROS 2 Jazzy - Robot Padi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Node utama controller
            'swerve_drive_controller = swerve_controller.swerve_drive_controller:main',
            # Publisher trajectory
            'trajectory_publisher = swerve_controller.trajectory_publisher:main',
        ],
    },
)
