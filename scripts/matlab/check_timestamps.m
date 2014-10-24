%timestamps_file = '/home/steve/Desktop/output-timestamps.txt';

version = 'workspace';

timestamps_files = {};

%timestamps_files{1} = '/home/steve/ros_laboratory/thermalvis-ros-pkg/thermalvis/nodes/streamer/log/images-timestamps.txt';

timestamps_files{1} = sprintf('/home/steve/ros_%s/thermalvis-ros-pkg/thermalvis/nodes/streamer/log/%s.txt', version, 'call_log');
timestamps_files{2} = sprintf('/home/steve/ros_%s/thermalvis-ros-pkg/thermalvis/nodes/streamer/log/%s.txt', version, 'retrieve_log');

%timestamps_file = '/home/steve/peyman/thermal-kinect/time.txt';

col{1} = 'r';

col{2} = 'k';
col{3} = 'b';
%col{4} = 'r';

close all;
figure(1);

B = load(timestamps_files{1});
min_B = min(B(:));
B2 = (B - min_B) / 1e9;

for iii = 2:size(timestamps_files, 2)
    A = load(timestamps_files{iii});
    min_A = min(A(:));

    %X = (A(1:size(B,1)) - B) / 1e9;
    X = (A(1:size(B,1)) - min_B) / 1e9;
    %A = (A(1:size(B,1)) - min(A(:))) / 1e9;

    %plot(B-A, col{iii});
    plot(X, col{iii});
    
    hold on
end


xlabel('frame #');
ylabel('time (s)');