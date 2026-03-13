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
            'ball_launcher = fyp_wamv_project.ball_launcher_controller_node:main',
            'HSV_filter = fyp_wamv_project.HSV_filter:main',
            'depth_map = fyp_wamv_project.depth_map:main',
            'shape_detection = fyp_wamv_project.shape_detection:main',
            'template_matching = fyp_wamv_project.template_matching:main',
            'ball_trajectory = fyp_wamv_project.ball_trajectory:main',
            'image_sampling = fyp_wamv_project.image_sampling:main',
            'extract_features = fyp_wamv_project.extract_hybrid_features:main',
        ],
    },
)
