from setuptools import setup

package_name = 'aeb_k'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'aeb_k/launch/aeb_k.launch.py',
            'aeb_k/launch/cone_visualizer.launch.py',
            'aeb_k/launch/aeb_k_with_visualizer.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taeyoon Kim',
    maintainer_email='tykim0721@naver.com',
    description='AEB Determination based on fused cones',
    license='KU',
    entry_points={
        'console_scripts': [
            'aeb_k = aeb_k.aeb_k:main',
            'cone_visualizer = aeb_k.cone_visualizer:main',
        ],
    },
)
