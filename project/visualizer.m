% CURRENTLY ONLY ACCEPTS .txt FILES WITH ONLY 1 INTEGER PER LINE
% NEEDS TO BE UPDATED ONCE .degree.txt file CONVENTION IS AGREED

% ------------- Speaker Visualizer Set-up --------------- %

% Load degree values from the file
fileID = fopen('degrees.txt', 'r');
angles_deg = fscanf(fileID, '%f');
fclose(fileID);

% Convert degrees to radians for the moving point
angles_rad = deg2rad(angles_deg);

% Circle properties
r = 1; % Radius of the circle
theta = linspace(0, 2*pi, 100);
x_circle = r * cos(theta);
y_circle = r * sin(theta);

% Define an array of degrees for stationary dots (modifiable)
stationary_deg = [0, 60, 120, 180, 240, 300]; 
stationary_rad = deg2rad(stationary_deg); 

% Calculate stationary dot coordinates
x_stationary = r * cos(stationary_rad);
y_stationary = r * sin(stationary_rad);

% Create a figure
figure;
plot(x_circle, y_circle, 'LineWidth', 2);
axis equal;
hold on;

% Plot stationary dots individually
num_stationary_dots = length(stationary_deg);
stationary_dots = gobjects(1, num_stationary_dots); % Preallocate array for plot objects
for i = 1:num_stationary_dots
    stationary_dots(i) = plot(x_stationary(i), y_stationary(i), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
end

% Initial point for moving dot
moving_dot = plot(r * cos(angles_rad(1)), r * sin(angles_rad(1)), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');


% ------------- Tracker Visualizer Set-up --------------- %
% Start scanner
rplidar_mex('setup', '/dev/ttyUSB0', 256000);
rplidar_mex('startScan');

% Plot scanner location
scatter(-600, 0, 30, 'o', 'filled');

% Update this point to show current track of participant
tracker_guess = plot(nan, nan, 'ro', 'MarkerSize', 10, 'MarkerFaceColor','green');

% Animation loop
for k = 1:length(angles_rad)

    % Update moving dot position
    x_moving = r * cos(angles_rad(k));
    y_moving = r * sin(angles_rad(k));
    set(moving_dot, 'XData', x_moving, 'YData', y_moving);

    % Calculate Euclidean distances between moving dot and each stationary dot
    distances = sqrt((x_stationary - x_moving).^2 + (y_stationary - y_moving).^2);

    % Find the indices of the two closest stationary dots, save 2 closest
    [~, sorted_indices] = sort(distances);
    closest_indices = sorted_indices(1:2); % Two closest dots

    % Update colors of stationary dots
    for i = 1:num_stationary_dots
        if ismember(i, closest_indices)
            set(stationary_dots(i), 'MarkerFaceColor', 'g'); % Highlight green
        else
            set(stationary_dots(i), 'MarkerFaceColor', 'b'); % Reset to blue
        end
    end

    try
        [~, ~, r, theta] = dbscan_test();
        set(tracker_guess, 'XData', r * cosd(theta), 'YData', r * sind(theta));
    catch
        warning('Participant not found.');
    end

    % CHANGE THIS TO MATCH REFRESH RATE
    pause(0.4);
end

rplidar_mex('close');

title('Matlab Visualizer Prototype');
xlabel('X');
ylabel('Y');
grid on;