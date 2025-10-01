from setuptools import setup

package_name = 'serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='you@example.com',
    description='ROS2 → Serial bridge for throttle & steering.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge = serial_bridge.serial_bridge:main',
        ],
    },
)

