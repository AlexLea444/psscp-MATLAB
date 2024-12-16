% Define the LIDAR SLAM object
maxRange = 23;       % Maximum range in meters
mapResolution = 20;   % Resolution in cells per meter
slamAlg = lidarSLAM(mapResolution, maxRange);

% Set loop closure parameters
slamAlg.LoopClosureThreshold = 210;
slamAlg.LoopClosureSearchRadius = 8;

% Initialize the scanner
rplidar_mex('setup', '/dev/ttyUSB0', 256000);
rplidar_mex('startScan');

% Process loop
for i = 1:10
    % Retrieve data from scanner
    [angles, distances, ~] = rplidar_mex('getData');

    % Filter for valid data
    validIdx = distances > 0 & distances < maxRange * 1000;

    % Convert to Cartesian, in meters
    xPoints = distances(validIdx) .* cosd(angles(validIdx)) / 1000;
    yPoints = distances(validIdx) .* sind(angles(validIdx)) / 1000;

    % Create LIDAR Scan object
    scan = lidarScan([xPoints, yPoints]);

    % Add Scan to SLAM algorithm
    isScanAccepted = addScan(slamAlg, scan);

    % Visualize SLAM progress
    if isScanAccepted
        figure(1);
        show(slamAlg);
        title('SLAM in Progress');
        drawnow;
    end
end

rplidar_mex('close');

figure;
show(slamAlg);
title('Generated Map using LIDAR SLAM');