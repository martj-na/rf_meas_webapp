from setuptools import setup
from glob import glob
import os

package_name = 'rf_meas_webapp'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dotX Automation',
    maintainer_email='info@dotxautomation.com',
    description='WebApp for RadioFrequency Measurements',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rf_meas_webapp = rf_meas_webapp.rf_meas_webapp_app:main',
        ],
    },
)
