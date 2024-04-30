from setuptools import find_packages, setup

package_name = 'port_pyqt'

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
    maintainer='zhu',
    maintainer_email='zzhu77@jhu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'port_ui = port_pyqt.port_pyqt:main',
            'test_ndi = port_pyqt.test_ndi:main',
            'pivot_cal = port_pyqt.pivot_cal:main',
            'test_pivot = port_pyqt.test_pivot:main',
        ],
    },
)
