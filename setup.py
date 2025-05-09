from setuptools import setup, find_packages

package_name = 'radar_sdk_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # radar_sdk_ros 내부 모듈 자동 탐색
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'ifxradarsdk'
    ],
    zip_safe=True,
    maintainer='juhun Jeong',
    maintainer_email='jjh011013@koreatech.ac.kr',
    description='레이더 SDK ROS2 패키지',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'campub = radar_sdk_ros.cam.campub:main',         # ✅ cam 디렉토리 경로로 수정
            'camsub = radar_sdk_ros.cam.camsub:main',
            'radar_pub_limit = radar_sdk_ros.radar_pub_limit:main',
            'radar_pub = radar_sdk_ros.radar_pub:main',
            'radar_sub = radar_sdk_ros.radar_sub:main',
            'radar_spectrogram = radar_sdk_ros.real_time_radar_sub:main',
        ],
    },
)
