from setuptools import find_packages, setup

package_name = 'docking_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		        'waypoint_mission = docking_pkg.waypoint_mission:main',
			'mission_manager = docking_pkg.mission_manager:main',
			'battery_node = docking_pkg.battery_node:main',
        ],
    },
)
