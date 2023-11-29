import os
from setuptools import setup
from glob import glob

package_name = 'composer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "log"), glob("log/*.log")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Composiv',
    maintainer_email='info@composiv.ai',
    description="Eclipse Muto's Composer component that is responsible for the lifecycle of ROS nodes",
    license='EPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'muto_composer = composer.muto_composer:main',
            'compose_plugin = composer.compose_plugin:main',
            'launch_plugin = composer.launch_plugin:main',
        ],
    },
)
