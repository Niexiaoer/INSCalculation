function Tracalculate(IMUdata)

    global Velocity Position Position_GPS 
%%%%%%%%%%%%%%%%��������%%%%%%%%%%%%%%%%%%
    Ra = 6378140;                                                             %6000000%����ο�������뾶
    e = 1/298.257;                                                            %�����������Բ��
    wie = 15.04088/3600/180*pi;                                               %������ת���ٶ�
    dt = 1/100;                                                               %����ʱ��
    
    accX = IMUdata(:,1);
    accY = IMUdata(:,2);
    accZ = IMUdata(:,3);
    Roll = IMUdata(:,4);
    Pitch = IMUdata(:,5);
    Yaw = IMUdata(:,6);
%%%%%%%%%%%%%%%�����������%%%%%%%%%%%%%%%%
    for i=1:size(IMUdata,1)
        C_bn = Rot_mat_Euler(Roll(i),Pitch(i),Yaw(i));                        %ŷ���Ǽ�������ϵת������bϵ��nϵ    
        C_bn = C_bn/(C_bn'*C_bn)^0.5;                                         %����������
        L   = Position_GPS(i,1);                                              %γ��
        Rx = (Ra+Position_GPS(i,3))*(1+e*sin(L)^2);                           %�ο�����������ķ���ƽ���ڵ����ʰ뾶����������
        Ry = (Ra+Position_GPS(i,3))*(1-2*e+3*e*sin(L)^2);                     %�ο������������ڵ����ʰ뾶����������
        g = 9.794617509364013074*(1+0.00193185138639*sin(L)^2)/sqrt(1-0.00669437999013*sin(L)^2);  %�������ٶ�
        w_ie =[0; wie*cos(L); wie*sin(L)];                                    %������ת�ڵ���ϵ�ڵķ���
        w_en =[-V_y/Ry; V_x/Rx ;V_x*tan(L)/Rx];                               %�����ص�������˶��Ľ��ٶ�
        
        Acc_f  = ([Ax,Ay,Az]')*g;
        Spec_f_n1 = C_bn'*Acc_f;                                                  %����ϵ���ٶȡ����������̵�һ���׼�����
        Spec_f_n2 = cross_mine(w_ie,w_en,V_velocity_0);                           %���ϼ��ٶȡ����������̵ڶ���
        Spec_f_n3 = [0;0;1]*g;                                                    %�������ٶ����������������̵�����
        Spec_Acc  = Spec_f_n1 - Spec_f_n2 + Spec_f_n3;                            %�������̼���õ�����ʵ���ٶ�
    end
end

