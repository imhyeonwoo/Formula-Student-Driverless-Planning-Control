from setuptools import setup

package_name = 'cmd_gate'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cmd_gate_only.launch.py', 'launch/traffic_light_mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cmd_gate_maintainer',
    maintainer_email='dev@example.com',
    description='Gate node that controls /cmd/* outputs based on /green service.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_gate_node = cmd_gate.gate_node:main',
        ],
    },
)

