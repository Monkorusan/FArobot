from setuptools import find_packages, setup

package_name = 'FArobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/boxes.yaml', 'config/packing_demo.rviz']),
        ('share/' + package_name + '/launch', [
            'launch/packing_demo.launch.py',
            'launch/packing_demo_ur5e_moveit.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='monkorusan',
    maintainer_email='chhuon.s.c359@m.isct.ac.jp',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'packing_demo_node = FArobot.packing_demo_node:main',
            'pick_place_executor_node = FArobot.pick_place_executor_node:main',
            'robot_description_publisher_node = FArobot.robot_description_publisher_node:main',
        ],
    },
)
