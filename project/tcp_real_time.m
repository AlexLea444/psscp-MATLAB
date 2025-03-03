rplidar_mex('setup', '/dev/ttyUSB0', 256000);
rplidar_mex('startScan');

% MATLAB TCP Server to Send User Position Data
t = tcpserver('127.0.0.1', 5002); % Open TCP server on port 5000
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
        try
            [x, y] = dbscan_global_position(R, M, delta);
            %[r, theta] = dbscan_test(R, M, delta);
            %theta = theta + 10;
            %pause(0.01);
        catch
            warning('Participant not found.');
        end
        
        % Send data as JSON
        jsonData = jsonencode(struct('radius', r, 'angle', theta));
    
        % COMMENT IN FOR AUDIO DATA SENDING PROTO
        % pause(2)
        % jsonData = jsonencode(struct('sounddegree', randi([0, 360])));
    
        jsonBytes = uint8([jsonData, newline]);
        write(t, jsonBytes); % Send as newline-terminated string
    catch ME
        disp(['Client stopped: ', ME.message]);
        cleanupServer(t);
        break;
    end
end

function cleanupServer(server)
    disp('Cleaning up and closing server...');
    
    try
        if isvalid(server)
            delete(server);  
        else
            disp('Server object is no longer valid.');
        end
        
        clear server;
        disp('Server object cleared from memory.');
        
    catch ME
        disp(['Error during cleanup: ', ME.message]);
    end
end

