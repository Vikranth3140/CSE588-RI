import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vikranth/turtlebot3_ws/src/vikranth_gazebo/install/vikranth_gazebo'
