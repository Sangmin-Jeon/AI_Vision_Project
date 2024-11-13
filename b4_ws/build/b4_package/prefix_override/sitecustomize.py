import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yeeun/AI_Vision_Project/b4_ws/install/b4_package'
