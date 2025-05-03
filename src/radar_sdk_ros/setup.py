from setuptools import setup

package_name = 'radar_sdk_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'ifxradarsdk'],
    zip_safe=True,
    maintainer='juhun Jeong',
    maintainer_email='jjh011013@koreatech.ac.kr',
    description='레이더 SDK ROS2 패키지',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'campub = radar_sdk_ros.campub:main',  # 퍼블리셔 campub
 	        'camsub = radar_sdk_ros.camsub:main',  # 서브스크라이버 camsub
            'radar_pub_limit = radar_sdk_ros.radar_pub_limit:main',
            'radar_pub = radar_sdk_ros.radar_pub:main',
            'radar_sub = radar_sdk_ros.radar_sub:main',
            'radar_spectrogram = radar_sdk_ros.real_time_radar_sub'
        ],
    },
)
