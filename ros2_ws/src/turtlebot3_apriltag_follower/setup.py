from setuptools import find_packages, setup

package_name = 'turtlebot3_apriltag_follower'

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
    maintainer='naomichen',
    maintainer_email='naomichen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'apriltag_follower_node = turtlebot3_apriltag_follower.apriltag_follower:main',
        ],
    },
)
