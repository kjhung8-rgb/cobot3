from launch import LaunchDescription
from launch.actions import ExecuteProcess

ISAAC_PYTHON = '/home/kim/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh'
SCRIPT_PATH  = '/home/kim/dev_ws/cobot3/src/yolo/spot_people_crowd2.py'

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[ISAAC_PYTHON, SCRIPT_PATH],
            cwd='/home/kim/dev_ws/cobot3/src/yolo',
            output='screen',
        ),
    ])
