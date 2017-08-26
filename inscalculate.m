function inscalculate(receiveData,dt)
    global Spec_Acc Velocity Position Position_GPS 
%%%%%%%%%%%%%%%%变量定义%%%%%%%%%%%%%%%%%%
    Ra = 6378140;                                                             %6000000%地球参考椭球长轴半径
    e = 1/298.257;                                                            %地球椭球的椭圆度
    wie = 15.04088/3600/180*pi;                                               %地球自转角速度
    V_velocity_0 = Velocity;                                                  %记录前一时刻的速度
    
    V_x = V_velocity_0(1);                                                    %x轴速度——东方向
    V_y = V_velocity_0(2);                                                    %y轴速度——北方向
    L   = Position_GPS(1);                                                    %纬度
   
    Rx = (Ra+Position_GPS(3))*(1+e*sin(L)^2);                                 %参考椭球子午面的法线平面内的曲率半径——东方向
    Ry = (Ra+Position_GPS(3))*(1-2*e+3*e*sin(L)^2);                           %参考椭球子午面内的曲率半径——北方向
    AccBias = [0;0;-0.02031];                                                  %水平坐标系加速度更正
    g = 9.794617509364013074*(1+0.00193185138639*sin(L)^2)/sqrt(1-0.00669437999013*sin(L)^2);  %重力加速度
%%%%%%%%%%%%%%%变量定义完毕%%%%%%%%%%%%%%%%

%%%%%%%%%%%%惯导解算%%%%%%%%%%%%%%%%%%%%%%%%
    Ax = receiveData(1);Ay = receiveData(2);Az = receiveData(3);
    Roll = receiveData(4);Pitch = receiveData(5);Yaw = receiveData(6);
    C_bn = Rot_mat_Euler(Roll,Pitch,Yaw);                                     %欧拉角计算坐标系转换矩阵，b系至n系                                             %四元数计算坐标系转换矩阵，b系至n系
    C_bn = C_bn/(C_bn'*C_bn)^0.5;                                             %保持正交化
    w_ie =[0; wie*cos(L); wie*sin(L)];                                        %地球自转在导航系内的分量
    w_en =[-V_y/Ry; V_x/Rx ;V_x*tan(L)/Rx];                                   %载体沿地球表面运动的角速度
    
    Acc_f = ([Ax,Ay,Az]')*g;
    Spec_f_n1 = C_bn'*Acc_f;                                                  %导航系加速度——比力方程第一项（标准输出）
    Spec_f_n2 = cross_mine(w_ie,w_en,V_velocity_0);                           %哥氏加速度——比力方程第二项
    Spec_f_n3 = [0;0;1]*g;                                                    %重力加速度修正——比力方程第三项
    Spec_Acc  = Spec_f_n1 - Spec_f_n2 + Spec_f_n3;                            %比力方程计算得到的真实加速度
  %  Spec_Acc  = Spec_Acc + AccBias;                                           %加速度更正
    
    Velocity = V_velocity_0 +  (Spec_Acc.*[1;1;-1])*dt;                       %导航速度计算
    d_Position = (V_velocity_0 + Velocity)*dt/2;                              %导航位置变化率计算
    Position   = Position + d_Position;                                       %导航位置计算
    
    d_Position_latitude  = d_Position(2)/Ry;                                  %当前位置纬度变化计算
    d_Position_longitude = d_Position(1)/Rx;                                  %当前位置经度变化计算
    d_Position_high      = d_Position(3);                                     %当前位置高度变化计算
    Position_GPS         = Position_GPS + [d_Position_latitude;d_Position_longitude;d_Position_high]; %当前位置坐标，[纬度，经度，高度]，弧度制
%%%%%%%%%%%%%%%惯导解算完毕%%%%%%%%%%%%%%%%%

%%  计算坐标系转换矩阵——惯导输出欧拉角计算 %%
function  out = Rot_mat_Euler(Roll_receieve,Pitch_receieve,Yaw_receieve)

an_x = Roll_receieve;                                                         %横滚角
an_y = Pitch_receieve;                                                        %俯仰角
an_z = Yaw_receieve;                                                          %航偏角

mat_x = [1,0,0;0,cos(an_x),sin(an_x);0,-sin(an_x),cos(an_x)];                 %三个角度转换矩阵计算
mat_y = [cos(an_y),0,-sin(an_y);0,1,0;sin(an_y),0,cos(an_y)];
mat_z = [cos(an_z),sin(an_z),0;-sin(an_z),cos(an_z),0;0,0,1];

mat_all  = mat_x*mat_y*mat_z;                                                 %欧拉角旋转矩阵 
       
out = mat_all;    
end
 
 %%  叉乘  %%
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