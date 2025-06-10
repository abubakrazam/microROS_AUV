from setuptools import find_packages, setup

package_name = 'control_pkg'

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
    maintainer='aasim',
    maintainer_email='aasim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "main_thruster_controller = control_pkg.main_thruster_controller:main",
            "planar_stabilizer = control_pkg.planar_stabilizer:main",
            "joy_controller = control_pkg.joy_controller:main",
            "depth_holder = control_pkg.depth_holder:main",
            "heading_stabilizer = control_pkg.heading_stabilizer:main",
            "position_holder = control_pkg.position_holder:main",
            "sync_io=control_pkg.sync_io:main"
        ],
    },
)
