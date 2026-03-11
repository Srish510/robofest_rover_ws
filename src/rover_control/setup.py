from setuptools import find_packages, setup

package_name = 'rover_control'

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
            'esp32_bridge = rover_control.esp32_bridge:main',
            'motor_interface = rover_control.motor_interface:main',
            'odometry_node = rover_control.odometry_node:main',
            'mock_esp32 = rover_control.mock_esp32:main',
        ],
    },
)
