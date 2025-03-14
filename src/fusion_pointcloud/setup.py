from setuptools import find_packages, setup

package_name = 'fusion_pointcloud'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eirik',
    maintainer_email='eirikdy@uia.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pointcloud_to_xyz = fusion_pointcloud.pointcloud_to_xyz:main',
        'xyz_tid_test = fusion_pointcloud.xyz_tid_test:main'
        ],
    },
)
