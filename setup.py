from setuptools import find_packages, setup

package_name = 'ros2_optical_flow'

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
    maintainer='michaelmitch2002',
    maintainer_email='michaelmitch2002@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["ros2_optical_flow = ros2_optical_flow.ros2_optical_flow:main"
        ],
    },
)
