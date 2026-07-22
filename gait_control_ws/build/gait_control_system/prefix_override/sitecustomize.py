import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/zhang/新加卷/wc/test/new-dianji/PC-test/gait_control_ws/install/gait_control_system'
