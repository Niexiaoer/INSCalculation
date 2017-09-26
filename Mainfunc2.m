tic
clear;
close all;
addpath('Quaternions');
filePath = 'Datasets/imudata15.xlsx';
IMUdata= xlsread(filePath);
clc
time = zeros(1,size(IMUdata,1));
dt = 1/100;
for i=1:size(IMUdata,1)
    time(i) = (i-1)*dt;
end

[Vel_whole,Att_whole,VelError_whole,AttError_whole] = TraceState(IMUdata,dt);
% Plot translational Attitude
Att_whole = Att_whole*180/pi;
AttError_whole = AttError_whole*180/pi;
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Attitude');
hold on;
plot(time, Att_whole(:,1), 'r');
plot(time, Att_whole(:,2), 'g');
plot(time, Att_whole(:,3), 'b');
title('Attitude');
xlabel('Time (s)');
ylabel('Attitude (rad)');
legend('Pitch', 'Roll', 'Azimuth');
hold off;

% Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, Vel_whole(:,1), 'r');
plot(time, Vel_whole(:,2), 'g');
plot(time, Vel_whole(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('E', 'N', 'U');
hold off;

% Plot translational Attitude_error
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Attitude_error');
hold on;
plot(time, AttError_whole(:,1), 'r');
plot(time, AttError_whole(:,2), 'g');
plot(time, AttError_whole(:,3), 'b');
title('Attitude_error');
xlabel('Time (s)');
ylabel('Attitude_error (rad)');
legend('Pitch', 'Roll', 'Azimuth');
hold off;

% Plot translational Velocity_error
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity_error');
hold on;
plot(time, VelError_whole(:,1), 'r');
plot(time, VelError_whole(:,2), 'g');
plot(time, VelError_whole(:,3), 'b');
title('Velocity_error');
xlabel('Time (s)');
ylabel('Velocity_error (m/s)');
legend('E', 'N', 'U');
hold off;
