from setuptools import setup

package_name = 'taflab_msgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/ControlData.msg', 'msg/CalibrationData.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boat',
    maintainer_email='your-email@example.com',
    description='Message definitions for taflab project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
