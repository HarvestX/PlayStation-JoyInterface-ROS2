"""Setup."""
from glob import glob

from setuptools import setup

package_name = 'p9n_bringup'

setup(
    name=package_name,
    version='1.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/{}/launch'.format(package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m12watanabe1a',
    maintainer_email='40206149+m12watanabe1a@users.noreply.github.com',
    description='PlayStation Interface Node Launch Files.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
