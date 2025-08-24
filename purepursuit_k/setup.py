from setuptools import setup

package_name = 'purepursuit_k'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['purepursuit_k/launch/purepursuit.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taeyoon Kim',
    maintainer_email='tykim0721@naver.com',
    description='Pure Pursuit Control by K',
    license='KU',
    entry_points={
        'console_scripts': [
            'purepursuit = purepursuit_k.purepursuit:main',
        ],
    },
)

