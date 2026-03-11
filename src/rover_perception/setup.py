from setuptools import find_packages, setup

package_name = 'rover_perception'

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
    maintainer='srish',
    maintainer_email='kingsrishxd@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'realsense_node = rover_perception.realsense_node:main',
            'lane_detector = rover_perception.lane_detector:main',
            'obstacle_detector = rover_perception.obstacle_detector:main',
            'depth_processor = rover_perception.depth_processor:main',
            'terrain_mapper = rover_perception.terrain_mapper:main',
            'qr_scanner = rover_perception.qr_scanner:main',
            'mock_camera = rover_perception.mock_camera:main',
        ],
    },
)
