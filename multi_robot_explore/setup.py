from setuptools import find_packages, setup

package_name = 'multi_robot_explore'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Dudas',
    maintainer_email='david.dudas@outlook.com',
    description='Python package multi_robot_explore',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore_map = multi_robot_explore.explore_map:main',
            'explore_map2 = multi_robot_explore.explore_map2:main',
        ],
    },
)
