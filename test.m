clear;
close all;

filePath = 'Datasets/imudata16.xlsx';
IMUdata= xlsread(filePath);
IMUdata = IMUdata(1:1000,:);
AccX = IMUdata(:,1);
AccY = IMUdata(:,2);
AccZ = IMUdata(:,3);
WX = IMUdata(:,4);
WY = IMUdata(:,5);
WZ = IMUdata(:,6);
AccX = abs(AccX - mean(AccX));
AccY = abs(AccY - mean(AccY));
AccZ = abs(AccZ - mean(AccZ));
WX = abs(WX - mean(WX));
WY = abs(WY - mean(WY));
WZ = abs(WZ - mean(WZ));

% xcorr(AccX);
[a1,b1] = xcorr(AccX,'none');
[a2,b2] = xcorr(AccY,'none');
[a3,b3] = xcorr(AccZ,'none');
[a4,b4] = xcorr(WX,'none');
[a5,b5] = xcorr(WY,'none');
[a6,b6] = xcorr(WZ,'none');

% dt = 1/100;
% b1 = [-10:10];
% plot(b1*dt,a1);
dt = 1/(100*3600);
figure;
subplot(3,1,1);
plot(b1*dt,a1);
xlabel('AccX');
subplot(3,1,2);
plot(b2*dt,a2);
xlabel('AccY');
subplot(3,1,3);
plot(b3*dt,a3);
xlabel('AccZ');

figure;
subplot(3,1,1);
plot(b4*dt,a4);
xlabel('Wx');
subplot(3,1,2);
plot(b5*dt,a5);
xlabel('Wy');
subplot(3,1,3);
plot(b6*dt,a6);
xlabel('Wz');
% AccX = IMUdata(:,1);
% 
% dt = 1/100;
% t = [0:dt:size(AccX,1)];
% [a,b] = xcorr(AccX,  'coeff'  );
% plot(b*dt,a);
