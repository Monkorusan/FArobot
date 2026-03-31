import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/monkorusan/ros2_ws/src/FArobot/install/FArobot'
