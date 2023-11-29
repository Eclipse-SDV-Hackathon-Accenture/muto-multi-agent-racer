from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'kuksa_ros_provider'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='composiv',
    maintainer_email='info@composiv.ai',
    description='ROS data provider for Eclipse Kuksa',
    license='EPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_provider = kuksa_ros_provider.ros_provider:main'
        ],
    },
)
