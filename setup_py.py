from setuptools import setup

package_name = 'vtem_control'

setup(
    name=package_name,
    version='0.9.4',
    packages=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Maximilian Stölzle',
    author_email='maximilian@stoelzle.ch',
    maintainer='Maximilian Stölzle',
    maintainer_email='maximilian@stoelzle.ch',
    keywords=['ROS'],
    description='Examples of minimal publishers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_pub = scripts.python_pub:main',
        ],
    },
)