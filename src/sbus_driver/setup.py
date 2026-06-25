from setuptools import find_packages, setup

package_name = 'sbus_driver'

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
    maintainer='jontro_soinik_2_0-2',
    maintainer_email='jontro_soinik_2_0-2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sbus_publisher = sbus_driver.sbus_publisher:main',
            'teleop_input_node = sbus_driver.teleop_input_node:main',
            'dummy_arm = sbus_driver.dummy_arm:main',
            'gpio_control = sbus_driver.gpio_control:main',
        ],
    },
)
