import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/thebigduke/colcon_ws/src/delivery_bridge/install/delivery_bridge'
