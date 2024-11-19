from setuptools import setup

package_name = 'my_gazebo_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.my_gazebo_scripts'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='My Gazebo simulation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_trpo_turtlebot = my_gazebo_simulation.my_gazebo_scripts.train_trpo_turtlebot:main',
        ],
    },
)
