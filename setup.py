from setuptools import setup
import os
from glob import glob

package_name = 'vip-hello-world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Chen',
    maintainer_email='achen694@gatech.edu',
    description='ROS2 package for color-based F1TENTH car control',
    license='TODO: License Declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = vip-hello-world.zed_camera_node:main',
            'processing_node = vip-hello-world.processing_node:main',
            'control_node = vip-hello-world.control_node:main',
        ],
    },
)
