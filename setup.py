from setuptools import find_packages, setup

package_name = 'ninja_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_ninja_turtle.py', 'launch/launch_draw_with_me.py', 'launch/launch_drawing_ninja.py']),
    ],
    install_requires=[
        'setuptools',
        'speech_recognition',
        'pyaudio',
        'pyttsx3'
    ],
    zip_safe=True,
    maintainer='manu',
    maintainer_email='sharmamanu2727@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'turtle_controller = ninja_turtle.turtle_controller:main',
            'command_publisher = ninja_turtle.command_publisher:main',
            'voice_listener = ninja_turtle.voice_listener:main',
            'dummy_voice_listener = ninja_turtle.dummy_voice_listener:main',
            'circle = ninja_turtle.circle:main',
            'game_manager = ninja_turtle.game_manager:main',
            'tts_node = ninja_turtle.tts_node:main',
        ],
    },
)
