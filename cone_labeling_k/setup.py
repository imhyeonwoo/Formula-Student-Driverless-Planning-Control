from setuptools import setup

package_name = 'cone_labeling_k'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['cone_labeling_k/launch/cone_labeling.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taeyoon Kim',
    maintainer_email='tykim0721@naver.com',
    description='Potential field based cone labeler',
    license='KU',
    entry_points={
        'console_scripts': [
            'potential_field = cone_labeling_k.potential_field:main',
        ],
    },
)

