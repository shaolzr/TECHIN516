from setuptools import find_packages, setup

package_name = 'lab_quaternion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/lab_quaternion/launch', ['launch/sort_world.launch.py']),
        ('share/lab_quaternion/worlds', ['worlds/sort_world.sdf']),
        ('share/lab_quaternion', ['lab_quaternion/data.csv']),
        ('share/lab_quaternion', ['lab_quaternion/data_pick.csv']),
        ('share/lab_quaternion', ['lab_quaternion/green_cube_put_data.csv']),
        ('share/lab_quaternion', ['lab_quaternion/green_cube_pick_data.csv']),
        ('share/lab_quaternion', ['lab_quaternion/grasp.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaolzr',
    maintainer_email='shaolzr@uw.edu',
    description='Lab for quaternion interpolation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'trajectory_from_csv = lab_quaternion.trajectory_from_csv:main',
        'gen3lite_pymoveit2 = lab_quaternion.gen3lite_pymoveit2:main',
        'sort_task = lab_quaternion.sort_task:main',
        'pick_and_put = lab_quaternion.pick_and_put:main',
        'grasp = lab_quaternion.grasp:main',

        ],
        
    },
)
