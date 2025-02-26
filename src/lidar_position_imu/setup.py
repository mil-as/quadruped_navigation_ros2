import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lidar_position_imu'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erlend',
    maintainer_email='erlendob@uia.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_position = lidar_position_imu.lidar_position:main'
        ],
    },
)
