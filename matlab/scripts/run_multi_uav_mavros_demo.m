function run_multi_uav_mavros_demo(N)
% RUN_MULTI_UAV_MAVROS_DEMO
% MATLAB ROS Toolbox example for controlling multiple UAVs via /uavX/mavros.
%
% Usage:
%   run_multi_uav_mavros_demo(3)

if nargin < 1
    N = 3;
end
if N < 1 || N > 10
    error('N must be in [1, 10].');
end

node = ros2node('/matlab_multi_uav_controller');
subsPose = cell(1, N);
subsImu = cell(1, N);
pubsSp = cell(1, N);

for i = 1:N
    ns = sprintf('/uav%d/mavros', i);
    subsPose{i} = ros2subscriber(node, [ns '/local_position/pose'], 'geometry_msgs/PoseStamped');
    subsImu{i} = ros2subscriber(node, [ns '/imu/data'], 'sensor_msgs/Imu');
    pubsSp{i} = ros2publisher(node, [ns '/setpoint_position/local'], 'geometry_msgs/PoseStamped');
end

ctrlRate = rateControl(20);
T = 30; % seconds
startT = tic;

fprintf('[MATLAB] Multi-UAV control start: N=%d\n', N);
while toc(startT) < T
    for i = 1:N
        poseMsg = receive(subsPose{i}, 0.05);
        imuMsg = receive(subsImu{i}, 0.05); %#ok<NASGU>

        target = ros2message(pubsSp{i});
        target.header.stamp = ros2time(node, 'now');

        % Keep each UAV at an offset hover point to avoid collisions.
        target.pose.position.x = double(i - 1) * 2.0;
        target.pose.position.y = 0.0;
        target.pose.position.z = max(1.5, double(poseMsg.pose.position.z));
        target.pose.orientation.w = 1.0;

        send(pubsSp{i}, target);
    end
    waitfor(ctrlRate);
end

fprintf('[MATLAB] Multi-UAV control finished.\n');
clear node;
end
