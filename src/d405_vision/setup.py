from setuptools import find_packages, setup

package_name = 'd405_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyberbraindualarm',
    maintainer_email='dongqiuyijing@163.com',
    description='D405 vision processing for FR3 manipulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixel_to_3d = d405_vision.pixel_to_3d:main',
            'depth_grayscale = d405_vision.depth_grayscale:main',
        ],
    },
)
