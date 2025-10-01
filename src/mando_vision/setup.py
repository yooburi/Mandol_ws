from setuptools import find_packages, setup

package_name = 'mando_vision'

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
    maintainer='highsky',
    maintainer_email='albert31115@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = mando_vision.vision_node:main',
            'unified_recorder = mando_vision.utils.unified_recorder:main',
        ],
    },
)
