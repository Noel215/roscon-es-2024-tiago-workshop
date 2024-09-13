import os
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'create_your_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noel Jimenez',
    maintainer_email='noel.jimenez@pal-robotics.com',
    description='Utils for the ROSCon Sevilla workshop - Tutorial de TIAGo en ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_motions_file = create_your_motion.load_motions_file:main'
        ],
    },
)
