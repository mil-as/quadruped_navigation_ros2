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
        'frontierExploration = autonomous_navigation.bfs_frontier_exploration_VI:main',
        'frontierNav = autonomous_navigation.frontier:main',
        'mapToMatrix = autonomous_navigation.map_to_matrix:main'
        ],
    },
)
