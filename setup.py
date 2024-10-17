from setuptools import setup

package_name = 'opencv_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oguzhan Demiroz',
    maintainer_email='oguzhan.demiroez@fau.de',
    description='Example package for image processing using OpenCV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = opencv_tools.image_publisher:main',
        ],
    },
)

