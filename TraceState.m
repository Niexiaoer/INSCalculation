function [Pos_whole,Vel_whole,Att_whole,PosError_whole,VelError_whole,AttError_whole]=TraceState(IMUdata,dt)
    addpath('Quaternions');
    %%%%%%%%%%%%%%�̶���������%%%%%%%%%%%%%%
    dGyro = [0;0;0];                                                       %�����ǹ̶���Ư
    bAcc = [0;0;0];                                                        %���ٱ�̶���Ư
    
    Pos_whole = zeros(size(IMUdata,1),3);
    Vel_whole = zeros(size(IMUdata,1),3);
    Att_whole = zeros(size(IMUdata,1),3); 
    PosError_whole = zeros(size(IMUdata,1),3);
    VelError_whole = zeros(size(IMUdata,1),3);
    AttError_whole = zeros(size(IMUdata,1),3);
    
    Q_wg  = (0.005*pi/180)^2;                      						   %���ݼư�����
    Q_wa  = (100e-6*9.7935)^2;                                             %���ټư�����
    Q     = diag([Q_wg Q_wg Q_wg, Q_wa Q_wa Q_wa]);                        %ϵͳ����Э����
    
    G = zeros(15,6);                                                       
    G(10:12,1:3) = eye(3,3);
    G(13:15,4:6) = eye(3,3);
    R = diag([1e-7,1e-7,1e-7]);                                            %�۲ⷽ����������R
    
    Re 		= 6378245;   %���򳤰뾶
    e 		= 1/298.257;  %�������
    wie 	= 7.292e-5;  %������ת���ٶ�
    g = 9.793563;                                                           %�����������ٶ�
    %%%%%%%%%%%%%%�̶���������%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%ϵͳ״̬������ʼ��%%%%%%%%%%%%%%
    Velocity = [0;0;0];
    VelocityChaRat = [0;0;0];
    Attitude = [-0.012574316;-0.000579696;-0.0017485];
    AttitudeChaRat = [0;0;0;0];
    Position_GPS = [30.5273*pi/180;114.3551*pi/180;32.43]; 
    ErrorState = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
    ErrorConv = diag([0.0001*pi/180 0.0001*pi/180 0.1,0.1 0.1 0.1,pi/180 pi/180 pi/180,0.01*pi/180 0.01*pi/180 0.01*pi/180,0.1 0.1 0.1]);
    %%%%%%%%%%%%%%ϵͳ״̬������ʼ��%%%%%%%%%%%%%%
    
    for i=1:size(IMUdata,1)
        %������ֵ
        Rm = Re*(1-2*e+3*e*sin(Position_GPS(1))^2)+Position_GPS(3);
        Rn = Re*(1-e*sin(Position_GPS(1))^2)+Position_GPS(3);
        wiel = [0;wie*cos(Position_GPS(1));wie*sin(Position_GPS(1))];      %Wie��Lϵ�ı�ʾ
        well = [-Velocity(2)/Rm;Velocity(1)/Rn;Velocity(1)*tan(Position_GPS(1))/Rn];%Wel��Lϵ�ı�ʾ
   
        %IMU�������
        Accb = IMUdata(i,1:3)';
        Gyro = IMUdata(i,4:6)';
        
        %%%%%%%%%%%%%%%%%%%%%%%�ߵ�����%%%%%%%%%%%%%%%%%%%%%%%%
        Rbl_0 = euler2rotMat(Attitude(1),Attitude(2),Attitude(3));
        QA_0 = (rotMat2quatern(Rbl_0))';
        QA_0 = normalize(QA_0);                                            %��һ�̵���̬��Ԫ��
        QA_1 = QA_0+AttitudeChaRat*dt;                                         
        QA_1 = normalize(QA_1);                                                %��ʱ�̵���̬��Ԫ��
        Rbl_1 = quatern2rotMat(QA_1');                                           %��ʱ�̵���̬����
        Attitude = (rotMat2euler(Rbl_1))';                                       %���¸�ʱ�̵���̬�Ƕ�
        
        VelocityChaRat_1 = Rbl_1*(Accb-bAcc)*g-cross_mine(2*wiel+well,Velocity)+[0;0;g];%��ʱ�̵��ٶȱ仯��
        Velocity_1 = Velocity+0.5*(VelocityChaRat+VelocityChaRat_1)*dt;    %��ʱ�̵��ٶ�
        
        dPosition = 0.5*(Velocity_1+Velocity)*dt;                              %λ��Lϵ�仯
        Position_GPS(3) = Position_GPS(3)+dPosition(3);                        %����Lϵγ�ȣ����ȣ��߶�����   
        Position_GPS(2) = Position_GPS(2)+dPosition(2)/(Rn*cos(Position_GPS(1)));  
        Position_GPS(1) = Position_GPS(1)+dPosition(1)/Rm; 
 
        VelocityChaRat = VelocityChaRat_1;                                     %�����ٶȱ仯��
        Velocity = Velocity_1;                                                 %�����ٶ�
        
        xitaib = Gyro*dt-dGyro*dt;  
        xitail = Rbl_1'*(wiel+well)*dt;
        xitalb = xitaib-xitail;
        omigalb = [0 xitalb(3) -xitalb(2) xitalb(1);
                   -xitalb(3) 0 xitalb(1) xitalb(2);
                   xitalb(2) -xitalb(1) 0 xitalb(3);
                   -xitalb(1) -xitalb(2) -xitalb(3) 0];
        AttitudeChaRat = 0.5*omigalb*QA_1;                                %������̬��Ԫ���仯��
        
        %ϵͳ״̬����
        Position_GPS = Position_GPS - ErrorState(1:3);
        Velocity = Velocity - ErrorState(4:6);
        Attitude = Attitude - ErrorState(7:9);
        Pos_whole(i,:) = Position_GPS';
        Vel_whole(i,:) = Velocity';
        Att_whole(i,:) = Attitude';
        %%%%%%%%%%%%%%%%%%%%%%%�ߵ�����%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%�ƶ����˲�%%%%%%%%%%%%%%%%%%%%%%%%    
        %����ϵͳ״̬ת���� F ��ʱ�����
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
    
        %����ϵͳ���������
        H 	 = zeros(3,15);
        H(1,4) = 1;
        H(2,5) = 1;
        H(3,6) = 1;
    
        %����ϵͳ��ɢ��
        Fai = eye(15)+F*dt;
        Gk = (eye(15,15)+dt*F/2)*G*dt;
    
        %�۲�ֵ����
        Z = Velocity;
        %�������˲�
        ErrorState_pri = Fai*ErrorState;                                       %�������˲���һ���
        ErrorConv_pri = Fai*ErrorConv*Fai'+ Gk*Q*Gk';                          %�������˲��ڶ����
        KalGain = ErrorConv_pri*H'*inv(H*ErrorConv_pri*H'+ R);                    %�������˲��������
        ErrorState = ErrorState_pri+KalGain*(Z-H*ErrorState_pri);              %�������˲��������
        ErrorConv = (eye(15,15)-KalGain*H)*ErrorConv_pri;                      %�������˲��������
        PosError_whole(i,:) = ErrorState(1:3)';
        VelError_whole(i,:) = ErrorState(4:6)';
        AttError_whole(i,:) = ErrorState(7:9)';
        %%%%%%%%%%%%%%%%%%%%%%%�ƶ����˲�%%%%%%%%%%%%%%%%%%%%%%%%  
    end   
     %%  ���  %%
function out = cross_mine(V1,V2)
R1 = [0 -V1(3) V1(2);
      V1(3) 0 -V1(1);
      -V1(2) V1(1) 0];
out   = R1*V2;
end

     %% ��������Ԫ�� %%
function Qout = normalize(Q)
delta = 1-(Q(1)^2+Q(2)^2+Q(3)^2+Q(4)^2);
Qout   = Q*(1+delta/2);
end    
end