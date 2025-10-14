from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=  [package_name], #find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dagl',
    maintainer_email='ashishm2611@gmail.com',
    description='Just a helper function to use fast-lio Odometry in Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'relay_topics = slam_nav.relay_topics:main',
        'amiga_node = slam_nav.amiga_node:main',
        'vel_node = slam_nav.vel_node:main',
        ],
    },
)
