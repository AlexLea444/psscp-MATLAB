rplidar_mex('setup', '/dev/ttyUSB0', 256000);
rplidar_mex('startScan');

% MATLAB TCP Server to Send User Position Data
t = tcpserver('127.0.0.1', 5000); % Open TCP server on port 5000
disp("Starting TCP Server. Awaiting handshake...");

% Wait for a message from the client
while t.NumBytesAvailable == 0
    pause(0.01);  % Wait for incoming data
end
readline(t)  % Read the client's confirmation message

R = 1000;
M = 200 + R;
delta = 200;

r = 0;
%r = 500;
theta = 0;

while true
    try
        [r, theta] = dbscan_test(R, M, delta);
        %theta = theta + 10;
        %pause(0.01);
    catch
        warning('Participant not found.');
    end
    
    % Send data as JSON
    jsonData = jsonencode(struct('radius', r, 'angle', theta));
    jsonBytes = uint8([jsonData, newline]);
    write(t, jsonBytes); % Send as newline-terminated string
end