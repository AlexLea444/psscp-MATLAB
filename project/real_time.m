rplidar_mex('setup', '/dev/ttyUSB0', 256000);
rplidar_mex('startScan');

figure;

R = 1000;
M = 200 + R;
delta = 200;

theta = linspace(0,2*pi,100);
plot(R * cos(theta), R * sin(theta), 'Linewidth', 2);
axis equal;
hold on;

% Overlay key object location
scatter(0, M, 30, 'o', 'filled'); % Larger green dot for scanner
title('LIDAR Scan with Key Object Overlay');
xlabel('X (mm)');
ylabel('Y (mm)');
legend('Target Scan Area', 'LIDAR Data Points', 'Target Center Guess', 'Scanner Location');
axis equal;
grid on;

tracker_guess = plot(nan, nan, 'ro', 'MarkerSize', 10, 'MarkerFaceColor','green');

while true
    try
        [r, theta] = dbscan_test(R, M, delta);
        set(tracker_guess, 'XData', r * cosd(theta), 'YData', r * sind(theta));
    catch
        warning('Participant not found.');
    end
    pause(0.4);
end
