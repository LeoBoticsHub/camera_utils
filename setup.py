from setuptools import setup

package_name = 'camera_utils'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[],
    install_requires=['setuptools', 'numpy', 'open3d', 'pyrealsense2', 'imutils'],
    zip_safe=True,
    maintainer='frollo',
    maintainer_email='rollo.f96@gmail.com',
    description='This package contains functions useful for camera use and other computer vision computations',
    license='GNU GENERAL PUBLIC LICENSE v3',
    entry_points={},
)
