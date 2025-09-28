import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/purvanya/projects/IMU-ESP/install/kalman_estimator'
