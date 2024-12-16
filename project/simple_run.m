rplidar_mex('setup', '/dev/ttyUSB0', 256000);
rplidar_mex('startScan');
[angles, distances, qualities] = rplidar_mex('getData');
rplidar_mex('close');