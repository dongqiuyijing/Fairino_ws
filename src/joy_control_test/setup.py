from setuptools import find_packages, setup

package_name = 'joy_control_test'

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
    maintainer='cyberbraindualarm',
    maintainer_email='dongqiuyijing@163.com',
    description='Read-only FR3 SDK status test for later joystick control.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sdk_read_test = joy_control_test.sdk_read_test:main',
            'sdk_jog_x_test = joy_control_test.sdk_jog_x_test:main',
            'joystick_jog_test = joy_control_test.joystick_jog_test:main',
        ],
    },
)
