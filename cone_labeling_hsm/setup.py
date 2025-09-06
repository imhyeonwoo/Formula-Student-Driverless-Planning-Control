from setuptools import setup

package_name = 'cone_labeling_hsm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seungmin',
    maintainer_email='seungmin4987@naver.com',
    description='Python-based cone labeling and path planning package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 실행 명령어 등록
            'path_planning = cone_labeling_hsm.path_planning:main',
            'straight_path = cone_labeling_hsm.path_planning_for_straight:main'
        ],
    },
)

