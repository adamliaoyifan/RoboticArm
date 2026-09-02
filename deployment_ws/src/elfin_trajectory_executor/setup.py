from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'elfin_trajectory_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Trajectory execution node for Elfin S20 (sim + real)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_executor = elfin_trajectory_executor.trajectory_executor_node:main',
        ],
    },
)
