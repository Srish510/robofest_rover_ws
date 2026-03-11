from setuptools import find_packages, setup

package_name = 'rover_comm'

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
            'video_stream_node = rover_comm.video_stream_node:main',
            'map_stream_node = rover_comm.map_stream_node:main',
            'telemetry_node = rover_comm.telemetry_node:main',
        ],
    },
)
