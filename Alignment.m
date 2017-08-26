function Alignment(receiveData,dt)
addpath('Quaternions');
global ErrorState Position_GPS Velocity Attitude ErrorConv;

%%%%%%%%%%%%%%%%变量定义%%%%%%%%%%%%%%%%%%
Ra = 6378140;
e = 0.0818;
W = sqrt(1-e*e*sin(Position_GPS(1))^2);
M = Ra*(1-e*e)/(W*W*W);
N = Ra/W;
Rm = M+Position_GPS(3);
Rn = N+Position_GPS(3);
wie = 15.04088/3600/180*pi;    
g = 9.7803267715*(1+0.0052790414*sin(Position_GPS(1))^2+0.0000232718*sin(Position_GPS(1))^4)+(-0.000003087691089+0.000000004397731*sin(Position_GPS(1))^2)*Position_GPS(3)+0.000000000000721*Position_GPS(3)^2;  %重力加速度
Gyroalfa = [0;0;0];
Accalfa = [0;0;0];
%%%%%%%%%%%%%%%%变量定义%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%输入变量%%%%%%%%%%%%%%%%%%
Accb = receiveData(1:3)';
Gyro = receiveData(4:6)';
%%%%%%%%%%%%%%%%输入变量%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%构造F矩阵%%%%%%%%%%%%%%%%%%
F = zeros(15,15);  
R = euler2rotMat(Attitude(1),Attitude(2),Attitude(3));
F(1:3,1:3) = [0,0,-Velocity(2)/(Rm^2);
           Velocity(1)*sin(Position_GPS(1))/(Rn*cos(Position_GPS(1))^2),0,-Velocity(1)/(Rn*Rn*cos(Position_GPS(1)));
           0,0,0];
F(1:3,4:6) = [0,1/Rm,0;
           1/(Rn*cos(Position_GPS(1))),0,0;
           0,0,1];
F(4:6,1:3) = [2*wie*sin(Position_GPS(1))*Velocity(3)+2*wie*cos(Position_GPS(1))*Velocity(2)+Velocity(1)*Velocity(2)/(Rn*cos(Position_GPS(1))^2),0,0
           -2*wie*cos(Position_GPS(1))*Velocity(1)-Velocity(1)^2/(Rn*cos(Position_GPS(1))^2),0,0
           -2*wie*sin(Position_GPS(1))*Velocity(1),0,2*g/sqrt(M*N)];
F(4:6,4:6) = [-Velocity(3)/Rn+Velocity(2)*tan(Position_GPS(1))/Rn,2*wie*sin(Position_GPS(1))+Velocity(1)*tan(Position_GPS(1))/Rn;-2*wie*cos(Position_GPS(1))-Velocity(1)/Rn;
           -2*Wie*sin(Position_GPS(1))-2*Velocity(1)*tan(Position_GPS(1))/Rn,-Velocity(3)/Rm,-Velocity(2)/Rm;
           2*wie*cos(Position_GPS(1))+2*Velocity(1)/Rn,2*Velocity(2)/Rm,0];
F(4:6,7:9) = [0,Acc(3),-Acc(2);
           -Acc(3),0,Acc(1);
           Acc(2),-Acc(1),0];
F(4:6,10:12) = R;
F(7:9,1:3) = [0,0,-Velocity(2)/(Rm^2);
           wie*sin(Position_GPS(1)),0,Velocity(1)/(Rn^2);
           -wie*cos(Position_GPS(1))-Velocity(1)/(cos(Position_GPS(1))^2*Rn),0,Velocity(1)*tan(Position_GPS(1))/(Rn^2)];
F(7:9,4:6) = [0,1/Rm,0;
           -1/Rn,0,0;
           -tan(Position_GPS(1))/Rn,0,0];
F(7:9,7:9) = [0,wie*sin(Position_GPS(1))+Velocity(1)*tan(Position_GPS(1))/Rn,-wie*cos(Position_GPS(1))-Velocity(1)/Rn;
           -wie*sin(Position_GPS(1))-Velocity(1)*tan(Position_GPS(1))/Rn,0,-Velocity(2)/Rm;
           wie*cos(Position_GPS(1))+Velocity(1)/Rn,Velocity(2)/Rm,0];
F(7:9,10:12) = R;
F(10:12,10:12) = [-Gyroalfa(1);-Gyroalfa(2);-Gyroalfa(3)];
F(13:15,13:15) = [-Accalfa(1);-Accalfa(2);-Accalfa(3)];
F = eye(15)+F*dt;
G =
%%%%%%%%%%%%%%%%构造F矩阵%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%卡尔曼滤波%%%%%%%%%%%%%%%%%
ErrorState_pri = F*ErrorState;
ErrorConv_pri = F*P*F'+
%%%%%%%%%%%%%%%%卡尔曼滤波%%%%%%%%%%%%%%%%%
end