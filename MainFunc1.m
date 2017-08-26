clear;
close all;
addpath('Quaternions');
filePath = 'Datasets/imudata14.xlsx';
global Velocity VelocityChaRat Attitude AttitudeChaRat Position_GPS Position
IMUdata= xlsread(filePath);

Velocity = [0;0;0];
VelocityChaRat = [0;0;0];
Attitude = [0;0;0];
AttitudeChaRat = [0;0;0;0];
Position_GPS = [30.5277*pi/180;114.3566*pi/180;40]; 
Position = [0;0;0];


Attitude_whole = zeros(size(IMUdata,1),3);
Velocity_whole = zeros(size(IMUdata,1),3);
Position_whole = zeros(size(IMUdata,1),3);

dt = 1/100;
time = zeros(size(IMUdata,1),1);
for i=1:size(IMUdata,1)
    time(i) = (i-1)*dt;
    receiveData = IMUdata(i,:);
    TraceState(receiveData,dt);
    Attitude_whole(i,:) = Attitude';
    Velocity_whole(i,:) = Velocity';
    Position_whole(i,:) = Position'; 
end

% Plot translational accelerations
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Attitude');
hold on;
plot(time, Attitude_whole(:,1), 'r');
plot(time, Attitude_whole(:,2), 'g');
plot(time, Attitude_whole(:,3), 'b');
title('Attitude');
xlabel('Time (s)');
ylabel('Attitude (deg)');
legend('Pitch', 'Roll', 'Azimuth');
hold off;

% Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, Velocity_whole(:,1), 'r');
plot(time, Velocity_whole(:,2), 'g');
plot(time, Velocity_whole(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'PositionXYZ');
hold on;
plot(time, Position_whole(:,1), 'r');
plot(time, Position_whole(:,2), 'g');
plot(time, Position_whole(:,3), 'b');
title('PositionXYZ');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;