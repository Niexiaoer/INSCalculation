function [Vel_whole,Att_whole,VelError_whole,AttError_whole]=TraceState(IMUdata,dt)
    addpath('Quaternions');
    %%%%%%%%%%%%%%�̶���������%%%%%%%%%%%%%%
    dGyro = [0;0;0];                                                       %�����ǹ̶���Ư
    bAcc = [0;0;0];                                                        %���ٱ�̶���Ư
    
    Vel_whole = zeros(size(IMUdata,1),3);
    Att_whole = zeros(size(IMUdata,1),3); 
    VelError_whole = zeros(size(IMUdata,1),3);
    AttError_whole = zeros(size(IMUdata,1),3);
    
    Q_wg  = (0.005*pi/180)^2;                      						   %���ݼư�����
    Q_wa  = (100e-6*9.7935)^2;                                             %���ټư�����
    Q     = diag([Q_wg Q_wg Q_wg, Q_wa Q_wa Q_wa]);                        %ϵͳ����Э����
    
    R = diag([1e-16,1e-16,1e-16]);                                            %�۲ⷽ����������R
    
    Re 		= 6378245;   %���򳤰뾶
    e 		= 1/298.257;  %�������
    wie 	= 7.292e-5;  %������ת���ٶ�
    g = 9.793563;                                                           %�����������ٶ�
    %%%%%%%%%%%%%%�̶���������%%%%%%%%%%%%%%
    
%     %�����ʼ��̬��
%     Gyro_0 = (mean(IMUdata(1:1000,4:6)))'
%     Accb_0 = (mean(IMUdata(1:1000,1:3)))'
%     V_0 = cross_mine(Accb_0,Gyro_0)
%     Gyro_l = wiel
%     Accb_l = [0;0;-1]
%     V_l = cross_mine(Accb_l,Gyro_l)
%     Rbl_0 = inv([Accb_l';Gyro_l';V_l'])*[Accb_0';Gyro_0';V_0'];
% %    Attitude_0_test = (rotMat2euler(Rbl_0_test))'  
    
    %%%%%%%%%%%%%%ϵͳ״̬������ʼ��%%%%%%%%%%%%%%
    Velocity_0 = [0;0;0];
    VelocityChaRat_0 = [0;0;0];
    Attitude_0 = [-0.012574316;-0.000579696;0.145734];  %roll pitch yaw
    AttitudeChaRat_0 = [0;0;0;0]; 
    Position_GPS = [30.5273*pi/180;114.3551*pi/180;32.43];
    ErrorState = [0;0;0;0;0;0;0;0;0;0;0;0]; %error of Ve Vn Vu Pitch roll azimuth GyroErrors AccbErrors
    ErrorConv = diag([0.0001^2 0.0001^2 0.0001^2,(pi/6480)^2 (pi/6480)^2 (pi/180)^2,(0.05*pi/180)^2 (0.05*pi/180)^2 (0.05*pi/180)^2,0.02^2 0.02^2 0.02^2]);
    Rbl_0 = euler2rotMat(-Attitude_0(1),-Attitude_0(2),-Attitude_0(3));
    QA_0 = euler2quatern(-Attitude_0(1),-Attitude_0(2),-Attitude_0(3));
    QA_0 = normalize(QA_0); 
    Accb_0 = [0;0;0];
    %%%%%%%%%%%%%%ϵͳ״̬������ʼ��%%%%%%%%%%%%%%
    
    Rm = Re*(1-2*e+3*e*sin(Position_GPS(1))^2)+Position_GPS(3);
    Rn = Re*(1-e*sin(Position_GPS(1))^2)+Position_GPS(3);
    wiel = [0;wie*cos(Position_GPS(1));wie*sin(Position_GPS(1))];      %Wie��Lϵ�ı�ʾ
  
    for i=1:size(IMUdata,1)
        %IMU�������
        Accb_1 = IMUdata(i,1:3)';
        Gyro_1 = IMUdata(i,4:6)';
        
        %%%%%%%%%%%%%%%%%%%%%%%�ߵ�����%%%%%%%%%%%%%%%%%%%%%%%%
        QA_1 = QA_0+AttitudeChaRat_0;                                         
        QA_1 = normalize(QA_1);                   %��ʱ�̵���̬��Ԫ��
        Rbl_1 = quatern2rotMat(QA_1);            %��ʱ�̵���̬����
        Attitude_1 = (rotMat2euler(Rbl_1))';      %���¸�ʱ�̵���̬�Ƕ�
        
        Velocity_1 = Velocity_0+VelocityChaRat_0*dt;    %��ʱ�̵��ٶ�  

        %%%%%%%%%%%%%%%%%%%%%%%�ߵ�����%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%�ƶ����˲�%%%%%%%%%%%%%%%%%%%%%%%%    
        %����ϵͳ״̬ת���� F ��ʱ�����
        F = zeros(12,12);  
        F(1:3,1:3) = [-Velocity_0(3)/Rn+Velocity_0(2)*tan(Position_GPS(1))/Rn,2*wie*sin(Position_GPS(1))+Velocity_0(1)*tan(Position_GPS(1))/Rn,-2*wie*cos(Position_GPS(1))-Velocity_0(1)/Rn;
                      -2*wie*sin(Position_GPS(1))-2*Velocity_0(1)*tan(Position_GPS(1))/Rn,-Velocity_0(3)/Rm,-Velocity_0(2)/Rm;
                      2*wie*cos(Position_GPS(1))+2*Velocity_0(1)/Rn,2*Velocity_0(2)/Rm,0];
        F(1:3,4:6) = [0,Accb_0(3),-Accb_0(2);
                      -Accb_0(3),0,Accb_0(1);
                      Accb_0(2),-Accb_0(1),0];
        F(1:3,10:12) = Rbl_0;
        F(4:6,1:3) = [0,1/Rm,0;
                      -1/Rn,0,0;
                      -tan(Position_GPS(1))/Rn,0,0];
        F(4:6,4:6) = [0,wie*sin(Position_GPS(1))+Velocity_0(1)*tan(Position_GPS(1))/Rn,-wie*cos(Position_GPS(1))-Velocity_0(1)/Rn;
                      -wie*sin(Position_GPS(1))-Velocity_0(1)*tan(Position_GPS(1))/Rn,0,-Velocity_0(2)/Rm;
                      wie*cos(Position_GPS(1))+Velocity_0(1)/Rn,Velocity_0(2)/Rm,0];
        F(4:6,7:9) = Rbl_0;
        F(7:9,7:9) = [-1/300 0 0;0 -1/300 0;0 0 -1/300];
        F(10:12,10:12) = [-1/1000 0 0;0 -1/1000 0;0 0 -1/1000];
    
        %����ϵͳ���������
        H 	 = zeros(3,12);
        H(1,1) = 1;
        H(2,2) = 1;
        H(3,3) = 1;
        
        G = zeros(12,6);
        G(1:3,4:6) = Rbl_0;
        G(4:6,1:3) = Rbl_0;
        G(7:9,1:3) = eye(3,3);
        G(10:12,4:6) = eye(3,3);
    
        %����ϵͳ��ɢ��
        Fai = eye(12,12)+F*dt;
        Gk = (eye(12,12)+dt*F/2)*G*dt;
    
        %�۲�ֵ����
        Z = Velocity_1;
        %�������˲�
        ErrorState_pri = Fai*ErrorState;                                       %�������˲���һ���
        ErrorConv_pri = Fai*ErrorConv*Fai'+ Gk*Q*Gk';                          %�������˲��ڶ����
        KalGain = ErrorConv_pri*H'*inv(H*ErrorConv_pri*H'+ R);                    %�������˲��������
        ErrorState = ErrorState_pri+KalGain*(Z-H*ErrorState_pri);              %�������˲��������
        ErrorConv = (eye(12,12)-KalGain*H)*ErrorConv_pri;                      %�������˲��������
        %%%%%%%%%%%%%%%%%%%%%%%�ƶ����˲�%%%%%%%%%%%%%%%%%%%%%%%%  
              
         %������ʱ�̵��ٶȺ���̬
         Velocity_1 = Velocity_1-ErrorState(1:3); %������ĸ�ʱ�̵��ٶ�
         Attitude_1 = Attitude_1-ErrorState(4:6);%������ĸ�ʱ�̵���̬�Ƕ�;      
         Rbl_1 = euler2rotMat(Attitude_1(1),Attitude_1(2),Attitude_1(3)); %������ĸ�ʱ�̵���ת����;
         QA_1 = rotMat2quatern(Rbl_1);                                    %������ĸ�ʱ�̵���Ԫ��;
       
        %�����ʱ�̵��ٶȱ仯�ʺ���̬�仯��
        well = [-Velocity_1(2)/Rm;Velocity_1(1)/Rn;Velocity_1(1)*tan(Position_GPS(1))/Rn];%Wel��Lϵ�����±�ʾ
        VelocityChaRat_1 = Rbl_1*(Accb_1-bAcc)*g-cross_mine(2*wiel+well,Velocity_1)+[0;0;g];%��ʱ�̵��ٶȱ仯��
        xitaib = Gyro_1*dt-dGyro*dt;  
        xitail = Rbl_1'*(wiel+well)*dt;
        xitalb = xitaib-xitail;
        omigalb = [0 xitalb(3) -xitalb(2) xitalb(1);
                   -xitalb(3) 0 xitalb(1) xitalb(2);
                   xitalb(2) -xitalb(1) 0 xitalb(3);
                   -xitalb(1) -xitalb(2) -xitalb(3) 0];
        AttitudeChaRat_1 = 0.5*omigalb*QA_1;                                 %������̬��Ԫ���仯��
        
        %���±���
        Velocity_0 = Velocity_1;
        Attitude_0 = Attitude_1;
        Rbl_0 = Rbl_1;
        QA_0 = QA_1;
        VelocityChaRat_0 = VelocityChaRat_1;
        AttitudeChaRat_0 = AttitudeChaRat_1;
        Accb_0 = Accb_1;
        
        Vel_whole(i,:) = Velocity_0';
        Att_whole(i,:) = Attitude_0';
        VelError_whole(i,:) = ErrorState(1:3)';
        AttError_whole(i,:) = ErrorState(4:6)';

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