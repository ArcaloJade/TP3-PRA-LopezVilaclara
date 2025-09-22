import sys
if sys.prefix == '/Users/mateolopezv/miniforge3/envs/rosenv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/install/custom_code'
