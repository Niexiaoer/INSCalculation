function TraceState(receiveData,dt,Q,G,R)
    addpath('Quaternions');
    global Velocity VelocityChaRat Attitude AttitudeChaRat Position_GPS Position ErrorState ErrorConv
    
    %变量定义
    W = sqrt(1-0.0818*0.0818*sin(Position_GPS(1))^2);
    Rm = 6378140*(1-0.0818*0.0818)/(W*W*W)+Position_GPS(3);
    Rn = 6378140/W+Position_GPS(3);
    wie = 15.04088/3600/180*pi;                                            %地球自转角速度
    wiel = [0;wie*cos(Position_GPS(1));wie*sin(Position_GPS(1))];          %Wie在L系的表示
    well = [-Velocity(2)/Rm;Velocity(1)/Rn;Velocity(1)*tan(Position_GPS(1))/Rn];%Wel在L系的表示
    g = 9.7803267715*(1+0.0052790414*sin(Position_GPS(1))^2+0.0000232718*sin(Position_GPS(1))^4)+(-0.000003087691089+0.000000004397731*sin(Position_GPS(1))^2)*Position_GPS(3)+0.000000000000721*Position_GPS(3)^2;  %重力加速度                                                 
    dGyro = [0;0;0];                                                       %陀螺仪固定偏差
    bAcc = [0;0;0];                                                        %加速表固定偏差
    
    %输入变量
    Accb = receiveData(1:3)';
    Gyro = receiveData(4:6)';

    %%%%%%%%%%%%%%%%%%%%%%%惯导解算%%%%%%%%%%%%%%%%%%%%%%%%
    Rbl_0 = euler2rotMat(Attitude(1),Attitude(2),Attitude(3));
    QA_0 = (rotMat2quatern(Rbl_0))';
    QA_0 = normalize(QA_0);                                                %上一刻的姿态四元数
    QA_1 = QA_0+AttitudeChaRat*dt;                                         
    QA_1 = normalize(QA_1);                                                %该时刻的姿态四元数
    Rbl_1 = quatern2rotMat(QA_1');                                           %该时刻的姿态矩阵
    Attitude = (rotMat2euler(Rbl_1))';                                       %更新该时刻的姿态角度
    
    VelocityChaRat_1 = Rbl_1*Accb*g-cross_mine(2*wiel+well,Velocity)+[0;0;g];%该时刻的速度变化率
    Velocity_1 = Velocity+0.5*(VelocityChaRat+VelocityChaRat_1)*dt-bAcc*dt;%该时刻的速度
    
    dPosition = 0.5*(Velocity_1+Velocity)*dt;                              %位置L系变化
    
    Position = Position+dPosition;
    Position_GPS(3) = Position_GPS(3)+dPosition(3);                        %更新L系纬度，经度，高度坐标
    Position_GPS(2) = Position_GPS(2)+dPosition(2)/(Rn*cos(Position_GPS(1)));
    Position_GPS(1) = Position_GPS(1)+dPosition(1)/Rm;
    
    VelocityChaRat = VelocityChaRat_1;                                     %更新速度变化率
    Velocity = Velocity_1;                                                 %更新速度

    xitaib = Gyro*dt-dGyro*dt;  
    xitail = Rbl_1*[-Velocity(2)/Rm;Velocity(1)/Rn+wie*cos(Position(1));Velocity(1)*tan(Position(1))/Rn+wie*sin(Position(1))]*dt;
    xitalb = xitaib-xitail;
    omigalb = [0 xitalb(3) -xitalb(2) xitalb(1);
               -xitalb(3) 0 xitalb(1) xitalb(2);
               xitalb(2) -xitalb(1) 0 xitalb(3);
               -xitalb(1) -xitalb(2) -xitalb(3) 0];
    AttitudeChaRat = QA_1+0.5*omigalb*QA_1;                                %更新姿态四元树变化率
    %系统状态更新
    Position_GPS = Position_GPS - ErrorState(1:3);
    Velocity = Velocity - ErrorState(4:6);
    Attitude = Attitude - ErrorState(7:9);
    %%%%%%%%%%%%%%%%%%%%%%%惯导解算%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%科尔曼滤波%%%%%%%%%%%%%%%%%%%%%%%%    
    %连续系统状态转换阵 F 的时间更新
    F = zeros(15,15);  
    F(1:3,1:3) = [0,0,-Velocity(2)/(Rm^2);
                  Velocity(1)*sin(Position_GPS(1))/(Rn*cos(Position_GPS(1))^2),0,-Velocity(1)/(Rn*Rn*cos(Position_GPS(1)));
                  0,0,0];
    F(1:3,4:6) = [0,1/Rm,0;
                  1/(Rn*cos(Position_GPS(1))),0,0;
                  0,0,1];
    F(4:6,1:3) = [2*wie*sin(Position_GPS(1))*Velocity(3)+2*wie*cos(Position_GPS(1))*Velocity(2)+Velocity(1)*Velocity(2)/(Rn*cos(Position_GPS(1))^2),0,0
                  -2*wie*cos(Position_GPS(1))*Velocity(1)-Velocity(1)^2/(Rn*cos(Position_GPS(1))^2),0,0
                  -2*wie*sin(Position_GPS(1))*Velocity(1),0,2*g/sqrt(Rm*Rn)];
    F(4:6,4:6) = [-Velocity(3)/Rn+Velocity(2)*tan(Position_GPS(1))/Rn,2*wie*sin(Position_GPS(1))+Velocity(1)*tan(Position_GPS(1))/Rn,-2*wie*cos(Position_GPS(1))-Velocity(1)/Rn;
                  -2*wie*sin(Position_GPS(1))-2*Velocity(1)*tan(Position_GPS(1))/Rn,-Velocity(3)/Rm,-Velocity(2)/Rm;
                  2*wie*cos(Position_GPS(1))+2*Velocity(1)/Rn,2*Velocity(2)/Rm,0];
    F(4:6,7:9) = [0,Accb(3),-Accb(2);
                  -Accb(3),0,Accb(1);
                  Accb(2),-Accb(1),0];
    F(4:6,10:12) = Rbl_1;
    F(7:9,1:3) = [0,0,-Velocity(2)/(Rm^2);
                  wie*sin(Position_GPS(1)),0,Velocity(1)/(Rn^2);
                  -wie*cos(Position_GPS(1))-Velocity(1)/(cos(Position_GPS(1))^2*Rn),0,Velocity(1)*tan(Position_GPS(1))/(Rn^2)];
    F(7:9,4:6) = [0,1/Rm,0;
                  -1/Rn,0,0;
                  -tan(Position_GPS(1))/Rn,0,0];
    F(7:9,7:9) = [0,wie*sin(Position_GPS(1))+Velocity(1)*tan(Position_GPS(1))/Rn,-wie*cos(Position_GPS(1))-Velocity(1)/Rn;
                  -wie*sin(Position_GPS(1))-Velocity(1)*tan(Position_GPS(1))/Rn,0,-Velocity(2)/Rm;
                  wie*cos(Position_GPS(1))+Velocity(1)/Rn,Velocity(2)/Rm,0];
    F(7:9,10:12) = Rbl_1;
    F(10:12,10:12) = [-1/300 0 0;0 -1/300 0;0 0 -1/300];
    F(13:15,13:15) = [-1/1000 0 0;0 -1/1000 0;0 0 -1/1000];
    
    %连续系统量测阵更新
    H 	 = zeros(3,15);
    H(1,4) = 1;
    H(2,5) = 1;
    H(3,6) = 1;
    
    %连续系统离散化
    Fai = eye(15)+F*dt;
    Gk = (eye(15,15)+dt*F/2)*G*dt;
    
    %观测值更新
    Z = Velocity;
    %卡尔曼滤波
    ErrorState_pri = Fai*ErrorState;                                       %卡尔曼滤波第一项方程
    ErrorConv_pri = Fai*ErrorConv*Fai'+ Gk*Q*Gk';                          %卡尔曼滤波第二项方程
    KalGain = ErrorConv_pri*H'*(H*ErrorConv_pri*H'+ R);                    %卡尔曼滤波第三项方程
    ErrorState = ErrorState_pri+KalGain*(Z-H*ErrorState_pri);              %卡尔曼滤波第四项方程
    ErrorConv = (eye(15,15)-KalGain*H)*ErrorConv_pri;                      %卡尔曼滤波第五项方程
%%%%%%%%%%%%%%%%%%%%%%%科尔曼滤波%%%%%%%%%%%%%%%%%%%%%%%%       
    
     %%  叉乘  %%
function out = cross_mine(V1,V2)
R1 = [0 -V1(3) V1(2);
      V1(3) 0 -V1(1);
      -V1(2) V1(1) 0];
out   = R1*V2;
end

     %% 正交化四元数 %%
function Qout = normalize(Q)
delta = 1-(Q(1)^2+Q(2)^2+Q(3)^2+Q(4)^2);
Qout   = Q*(1+delta/2);
end    
end