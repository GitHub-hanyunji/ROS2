import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rclpy_project4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'rclpy_project4', 'launch'), glob('launch/*.py')),
        # = 'share/rclpy_project4/launch'
        # 설치될 목적지 경로
        # = ['launch/my_launch.py', ...]
        # 현재 패키지 폴더에서 가져올 파일들
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linux',
    maintainer_email='doll90287@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pub=rclpy_project4.pub:main',
            'sub=rclpy_project4.sub:main',
        ],
    },
)
