from setuptools import find_packages, setup

package_name = 'wamv_logger'

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
    maintainer='tomas',
    maintainer_email='tocococa@gmail.com',
    description='Radio status and position logger for a Wamv ASV',
    license='unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wamv_logger = wamv_logger.wamv_logger_function:main',
        ],
    },
)
