function [object_distance, object_angle, r, theta] = localize()
    % Constants (Adjust these based on your scenario)
    R = 1 * 1000;  % Radius threshold
    M = R + 0.5 * 1000; % Distance from scanner to center of circle
    delta = 50; % Tolerance

    % Get LIDAR data
    [angles, distances, ~] = rplidar_mex('getData');

    % Initialize bit mask
    detection_bitmap = false(size(distances));

    % Update detection bitmap for circle approximation
    for i = 1:length(angles)
        detection_bitmap(i) = distances(i) < M * cosd(angles(i)) + sqrt(R^2 - M^2 * sind(angles(i))^2) - delta;
    end

    % Check that at least one point was detected
    if ~any(detection_bitmap)
        error('No valid indices found. Adjust your parameters or check LIDAR data.');
    end

    % Find the largest cluster of indices
    object_indices = refine(detection_bitmap);

    % Calculate median angle and distance for the identified object
    object_angle = median(angles(object_indices));
    object_distance = median(distances(object_indices));

    % Calculate the values for r and theta
    r = sqrt(M^2 - 2 * M * object_distance * cosd(object_angle) + object_distance^2);
    theta = acosd((M - object_distance * cosd(object_angle)) / sqrt(M^2 - 2 * M * object_distance * cosd(object_angle)));

    % Call visualization function to overlay object location on LIDAR scan
    visualizeLidarScan(angles(detection_bitmap), distances(detection_bitmap), object_angle, object_distance);
end

function object_indices = refine(detection_bitmap)
    % Transpose required for DBSCAN operation
    indices = find(detection_bitmap);

    % Parameters for DBSCAN
    epsilon = 1.5;
    minPts = 3;

    % Cluster by proximity
    idx = dbscan(indices, epsilon, minPts);

    % Largest cluster
    max_group = mode(idx(idx ~= -1));

    % Return largest cluster of object indices
    object_indices = transpose(indices(idx == max_group));
end

function visualizeLidarScan(angles, distances, object_angle, object_distance)
    % Convert LIDAR polar coordinates to Cartesian coordinates
    x = distances .* cosd(angles);
    y = distances .* sind(angles);

    % Convert key object's polar coordinates to Cartesian
    obj_x = object_distance * cosd(object_angle);
    obj_y = object_distance * sind(object_angle);

    % Plot LIDAR points
    figure;
    scatter(x, y, 5, 'b', 'filled');
    hold on;

    % Overlay key object location
    scatter(obj_x, obj_y, 60, 'r', 'filled'); % Larger red dot for object
    title('LIDAR Scan with Key Object Overlay');
    xlabel('X (mm)');
    ylabel('Y (mm)');
    legend('LIDAR Data Points', 'Key Object');
    axis equal;
    grid on;
    hold off;
end
