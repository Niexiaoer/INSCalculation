tic
clear;
close all;
addpath('Quaternions');
filePath = 'Datasets/imudata15.xlsx';
global Velocity VelocityChaRat Attitude AttitudeChaRat Position_GPS Position ErrorState ErrorConv
IMUdata= xlsread(filePath);

Velocity = [0;0;0];
VelocityChaRat = [0;0;0];
Attitude = [-0.012574316;-0.000579696;-0.0017485];
AttitudeChaRat = [0;0;0;0];
Position_GPS = [30.5273*pi/180;114.3551*pi/180;32.43]; 
ErrorState = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
ErrorConv = diag([0.0001*pi/180 0.0001*pi/180 0.1,0.1 0.1 0.1,pi/180 pi/180 pi/180,0.01*pi/180 0.01*pi/180 0.01*pi/180,0.1 0.1 0.1]);

Attitude_whole = zeros(size(IMUdata,1),3);
Velocity_whole = zeros(size(IMUdata,1),3);
Position_whole = zeros(size(IMUdata,1),3);
Attitude_error = zeros(size(IMUdata,1),3);
Velocity_error = zeros(size(IMUdata,1),3);
Position_Error = zeros(size(IMUdata,1),3);
dt = 1/100;

Q_wg  = (0.005*pi/180)^2;                      							   %陀螺计白噪声
Q_wa  = (100e-6*9.7935)^2;                                                 %加速计白噪声
Q     = diag([Q_wg Q_wg Q_wg, Q_wa Q_wa Q_wa]);                            %系统噪声协方差

Gyroalfa = [0;0;0];                                                        %陀螺仪误差传递函数参数
Accalfa = [0;0;0];                                                         %加速表误差传递函数参数
Gyroxita = [0;0;0];                                                        %陀螺仪误差传递方差
Accxita = [0;0;0];                                                         %加速表误差传递方差 
G = zeros(15,6);                                                       
G(10:12,1:3) = eye(3,3);
G(13:15,4:6) = eye(3,3);
            
R = diag([1e-6,1e-6,1e-6]);                                                %观测方程噪声方差R

time = zeros(1,size(IMUdata,1));
for i=1:size(IMUdata,1)
    time(i) = (i-1)*dt;
    receiveData = IMUdata(i,:);
    TraceState(receiveData,dt,Q,G,R);
    Attitude_whole(i,:) = Attitude';
    Velocity_whole(i,:) = Velocity';
    Position_whole(i,:) = Position'; 
    Attitude_error(i,:) = ErrorState(7:9)';
    Velocity_error(i,:) = ErrorState(4:6)';
    Position_Error(i,:) = ErrorState(1:3)';
end

% Plot translational Attitude
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Attitude');
hold on;
plot(time, Attitude_whole(:,1), 'r');
plot(time, Attitude_whole(:,2), 'g');
plot(time, Attitude_whole(:,3), 'b');
title('Attitude');
xlabel('Time (s)');
ylabel('Attitude (rad)');
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
legend('E', 'N', 'U');
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
legend('E', 'N', 'U');
hold off;

% Plot translational Attitude_error
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Attitude_error');
hold on;
plot(time, Attitude_error(:,1), 'r');
plot(time, Attitude_error(:,2), 'g');
plot(time, Attitude_error(:,3), 'b');
title('Attitude_error');
xlabel('Time (s)');
ylabel('Attitude_error (rad)');
legend('Pitch', 'Roll', 'Azimuth');
hold off;

% Plot translational Velocity_error
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity_error');
hold on;
plot(time, Velocity_error(:,1), 'r');
plot(time, Velocity_error(:,2), 'g');
plot(time, Velocity_error(:,3), 'b');
title('Velocity_error');
xlabel('Time (s)');
ylabel('Velocity_error (m/s)');
legend('E', 'N', 'U');
hold off;

% Plot translational Position_Error
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Position_Error');
hold on;
plot(time, Position_Error(:,1), 'r');
plot(time, Position_Error(:,2), 'g');
plot(time, Position_Error(:,3), 'b');
title('Position_Error');
xlabel('Time (s)');
ylabel('Position_Error');
legend('La', 'Lo', 'H');
hold off;
t = toc;