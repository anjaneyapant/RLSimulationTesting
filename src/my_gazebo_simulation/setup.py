from setuptools import setup

package_name = 'my_gazebo_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package for Gazebo simulations and RL training',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_trpo_turtlebot = my_gazebo_simulation.my_gazebo_scripts.train_trpo_turtlebot:main',
        ],
    },
)
