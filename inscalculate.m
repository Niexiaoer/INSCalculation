function inscalculate(receiveData,dt)
    global Spec_Acc Velocity Position Position_GPS 
%%%%%%%%%%%%%%%%��������%%%%%%%%%%%%%%%%%%
    Ra = 6378140;                                                             %6000000%����ο�������뾶
    e = 1/298.257;                                                            %�����������Բ��
    wie = 15.04088/3600/180*pi;                                               %������ת���ٶ�
    V_velocity_0 = Velocity;                                                  %��¼ǰһʱ�̵��ٶ�
    
    V_x = V_velocity_0(1);                                                    %x���ٶȡ���������
    V_y = V_velocity_0(2);                                                    %y���ٶȡ���������
    L   = Position_GPS(1);                                                    %γ��
   
    Rx = (Ra+Position_GPS(3))*(1+e*sin(L)^2);                                 %�ο�����������ķ���ƽ���ڵ����ʰ뾶����������
    Ry = (Ra+Position_GPS(3))*(1-2*e+3*e*sin(L)^2);                           %�ο������������ڵ����ʰ뾶����������
    AccBias = [0;0;-0.02031];                                                  %ˮƽ����ϵ���ٶȸ���
    g = 9.794617509364013074*(1+0.00193185138639*sin(L)^2)/sqrt(1-0.00669437999013*sin(L)^2);  %�������ٶ�
%%%%%%%%%%%%%%%�����������%%%%%%%%%%%%%%%%

%%%%%%%%%%%%�ߵ�����%%%%%%%%%%%%%%%%%%%%%%%%
    Ax = receiveData(1);Ay = receiveData(2);Az = receiveData(3);
    Roll = receiveData(4);Pitch = receiveData(5);Yaw = receiveData(6);
    C_bn = Rot_mat_Euler(Roll,Pitch,Yaw);                                     %ŷ���Ǽ�������ϵת������bϵ��nϵ                                             %��Ԫ����������ϵת������bϵ��nϵ
    C_bn = C_bn/(C_bn'*C_bn)^0.5;                                             %����������
    w_ie =[0; wie*cos(L); wie*sin(L)];                                        %������ת�ڵ���ϵ�ڵķ���
    w_en =[-V_y/Ry; V_x/Rx ;V_x*tan(L)/Rx];                                   %�����ص�������˶��Ľ��ٶ�
    
    Acc_f = ([Ax,Ay,Az]')*g;
    Spec_f_n1 = C_bn'*Acc_f;                                                  %����ϵ���ٶȡ����������̵�һ���׼�����
    Spec_f_n2 = cross_mine(w_ie,w_en,V_velocity_0);                           %���ϼ��ٶȡ����������̵ڶ���
    Spec_f_n3 = [0;0;1]*g;                                                    %�������ٶ����������������̵�����
    Spec_Acc  = Spec_f_n1 - Spec_f_n2 + Spec_f_n3;                            %�������̼���õ�����ʵ���ٶ�
  %  Spec_Acc  = Spec_Acc + AccBias;                                           %���ٶȸ���
    
    Velocity = V_velocity_0 +  (Spec_Acc.*[1;1;-1])*dt;                       %�����ٶȼ���
    d_Position = (V_velocity_0 + Velocity)*dt/2;                              %����λ�ñ仯�ʼ���
    Position   = Position + d_Position;                                       %����λ�ü���
    
    d_Position_latitude  = d_Position(2)/Ry;                                  %��ǰλ��γ�ȱ仯����
    d_Position_longitude = d_Position(1)/Rx;                                  %��ǰλ�þ��ȱ仯����
    d_Position_high      = d_Position(3);                                     %��ǰλ�ø߶ȱ仯����
    Position_GPS         = Position_GPS + [d_Position_latitude;d_Position_longitude;d_Position_high]; %��ǰλ�����꣬[γ�ȣ����ȣ��߶�]��������
%%%%%%%%%%%%%%%�ߵ��������%%%%%%%%%%%%%%%%%

%%  ��������ϵת�����󡪡��ߵ����ŷ���Ǽ��� %%
function  out = Rot_mat_Euler(Roll_receieve,Pitch_receieve,Yaw_receieve)

an_x = Roll_receieve;                                                         %�����
an_y = Pitch_receieve;                                                        %������
an_z = Yaw_receieve;                                                          %��ƫ��

mat_x = [1,0,0;0,cos(an_x),sin(an_x);0,-sin(an_x),cos(an_x)];                 %�����Ƕ�ת���������
mat_y = [cos(an_y),0,-sin(an_y);0,1,0;sin(an_y),0,cos(an_y)];
mat_z = [cos(an_z),sin(an_z),0;-sin(an_z),cos(an_z),0;0,0,1];

mat_all  = mat_x*mat_y*mat_z;                                                 %ŷ������ת���� 
       
out = mat_all;    
end
 
 %%  ���  %%
function out = cross_mine(w_ie,w_en,v)
w_ie_x = w_ie(1); w_ie_y = w_ie(2); w_ie_z = w_ie(3);
w_en_x = w_en(1); w_en_y = w_en(2); w_en_z = w_en(3);
%  v_x = v(1); v_y = v(2); v_z = v(3);
 
Mat_w = [0  -(2*w_ie_z+w_en_z) (2*w_ie_y+w_en_y);
         (2*w_ie_z+w_en_z) 0 -(2*w_ie_x+w_en_x);
         -(2*w_ie_y+w_en_y) (2*w_ie_x+w_en_x) 0;];
out   = Mat_w*v;
end

end