from setuptools import find_packages, setup

package_name = 'fyp_wamv_project'

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
    maintainer='kky',
    maintainer_email='kokyoongkang@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'binocular_vision = fyp_wamv_project.binocular_vision_node:main',
            'HSV_filter = fyp_wamv_project.HSV_filter:main',
            'depth_map = fyp_wamv_project.depth_map:main',
        ],
    },
)
