function [x, y] = dbscan_global_position(R, M, delta)
    % Constants (Adjust these based on your scenario)
    %R = 1 * 1000;  % Radius threshold
    %M = R + 0.2 * 1000; % Distance from scanner to center of circle
    %delta = 50; % Tolerance

    % Get LIDAR data
    [angles, distances, qualities] = rplidar_mex('getData');

    % Convert angles to [-180, 180] for use of sin^-1
    for i = 1:length(angles)
        if angles(i) > 180
            angles(i) = angles(i) - 360;
        end
        % ANGLES ARE FLIPPED
        angles(i) = -angles(i);
    end

    % Initialize bit mask
    detection_bitmap = false(size(distances));

    % Update detection bitmap for circle approximation
    for i = 1:length(angles)
        valid_distance = distances(i) < M * cosd(angles(i)) + sqrt(R^2 - M^2 * sind(angles(i))^2) + delta;
        valid_angle = (angles(i) > -asind(R / M)) & (angles(i) < asind(R / M));
        valid_quality = qualities(i) > 0;
        detection_bitmap(i) = valid_distance & valid_angle & valid_quality;
    end

    % Check that at least one point was detected
    if ~any(detection_bitmap)
        error('No valid indices found. Adjust your parameters or check LIDAR data.');
    end

    % Use dbscan to determine find the points associated with the participant
    coords = [distances(detection_bitmap) .* cosd(angles(detection_bitmap)), distances(detection_bitmap) .* sind(angles(detection_bitmap))];
    idx = dbscan(coords, 100, 5);

    % Largest cluster
    max_group = mode(idx(idx ~= -1));

    % Return largest cluster of object indices
    object_indices = (idx == max_group);

    % Calculate median angle and distance for the identified object
    object_x = (max(coords(object_indices)) + min(coords(object_indices))) / 2;
    object_y = (max(coords(object_indices, 2)) + min(coords(object_indices, 2))) / 2;
    
    x = 2.85 - (object_x / 1000);
    y = 1.905 + (object_y / 1000);
end