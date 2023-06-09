from setuptools import setup

package_name = 'pm_vision'
submodules = 'pm_vision/va_py_modules'
setup(
    #author='Niklas Terei',
    #author_email='terei@match.uni-hannover.de',
    #maintainer='Niklas Terei',
    #maintainer_email='terei@match.uni-hannover.de',
    name=package_name,
    version='0.1.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config/vision_assistant_path_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Niklas Terei',
    maintainer_email='terei@match.uni-hannover.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_assistant = pm_vision.vision_assistant_node:main',
            'vision_subscriber = pm_vision.vision_sub:main',
            'vision_webcam_publisher = pm_vision.webcam_image_pub:main',
        ],
    },
)
