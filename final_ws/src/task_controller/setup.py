from setuptools import find_packages, setup

package_name = 'task_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/task_controller', ['task_controller/put.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaolzr',
    maintainer_email='shaolzr@uw.edu',
    description='Task controller for final project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'run = task_controller.run:main',
        'move = task_controller.move:main',
        'back = task_controller.back:main',
        ],
    },
)
