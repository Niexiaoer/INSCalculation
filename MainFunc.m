clear;
close all;

addpath('Quaternions');

filePath = 'Datasets/imudata10.xlsx';
global Spec_Acc Velocity Position Position_GPS 
IMUdata= xlsread(filePath);


dt = 1/100;
Spec_Acc = [0;0;0];
Velocity = [0;0;0];
Position = [0;0;0];
Position_GPS = [30.5277*pi/180;114.3566*pi/180;40]; 
EulerAngle_whole = IMUdata(:,4:6);
Spec_Acc_whole = zeros(size(IMUdata,1),3);
Velocity_whole = zeros(size(IMUdata,1),3);
Position_whole = zeros(size(IMUdata,1),3);
time = zeros(size(IMUdata,1),1);
for i=1:size(IMUdata,1)
    time(i) = i*dt;
    receiveData = IMUdata(i,:);
    inscalculate1(receiveData,dt);
    Spec_Acc_whole(i,:) = Spec_Acc';
    Velocity_whole(i,:) = Velocity';
    Position_whole(i,:) = Position'; 
end

% Plot translational accelerations
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerations');
hold on;
plot(time, Spec_Acc_whole(:,1), 'r');
plot(time, Spec_Acc_whole(:,2), 'g');
plot(time, Spec_Acc_whole(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
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

% Create 6 DOF animation
SamplePlotFreq = 4;
Spin = 120;
SixDofAnimation(Position_whole, euler2rotMat(EulerAngle_whole(:,1),EulerAngle_whole(:,2),EulerAngle_whole(:,3)), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(Position_whole)-1)):(100+Spin))', 10*ones(length(Position_whole), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/dt) / dt));
