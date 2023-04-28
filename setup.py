from setuptools import setup

package_name = 'sas_datalogger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Murilo M. Marinho',
    maintainer_email='murilomarinho@ieee.org',
    description='TODO: Package description',
    license='LGPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sas_datalogger_node ='
            ' sas_datalogger.sas_datalogger_node:main',
        ],
    },
)
