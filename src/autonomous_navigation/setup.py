from setuptools import find_packages, setup

package_name = 'autonomous_navigation'

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
    maintainer='vegbrek',
    maintainer_email='vegbrek@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'costmap_to_image = autonomous_navigation.costmap_to_image:main',
        'costmap_viz = autonomous_navigation.costmap_viz:main',
        'costmap_testing = autonomous_navigation.costmap_testing:main',
        'frontierExploration = autonomous_exploration.frontier_exploration:main',
        'frontierExplorationTwo = autonomous_navigation.forntier_navigation2:main'
        ],
    },
)
