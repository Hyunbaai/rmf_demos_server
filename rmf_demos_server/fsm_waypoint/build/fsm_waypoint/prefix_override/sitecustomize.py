import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zeta/rmf_demos_server/fsm_waypoint/install/fsm_waypoint'
