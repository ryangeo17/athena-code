from setuptools import find_packages, setup

package_name = 'teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/ps4.yaml',
            'config/ps4_jetson.yaml',
            'config/thrustmaster.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duttaishan01',
    maintainer_email='duttaishan01@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick = teleop.joystick_publisher:main',
            'joy_remapper = teleop.joy_remapper:main',
        ],
    },
)
