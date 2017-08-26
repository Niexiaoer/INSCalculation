function Tracalculate(IMUdata)

    global Velocity Position Position_GPS 
%%%%%%%%%%%%%%%%变量定义%%%%%%%%%%%%%%%%%%
    Ra = 6378140;                                                             %6000000%地球参考椭球长轴半径
    e = 1/298.257;                                                            %地球椭球的椭圆度
    wie = 15.04088/3600/180*pi;                                               %地球自转角速度
    dt = 1/100;                                                               %采样时间
    
    accX = IMUdata(:,1);
    accY = IMUdata(:,2);
    accZ = IMUdata(:,3);
    Roll = IMUdata(:,4);
    Pitch = IMUdata(:,5);
    Yaw = IMUdata(:,6);
%%%%%%%%%%%%%%%变量定义完毕%%%%%%%%%%%%%%%%
    for i=1:size(IMUdata,1)
        C_bn = Rot_mat_Euler(Roll(i),Pitch(i),Yaw(i));                        %欧拉角计算坐标系转换矩阵，b系至n系    
        C_bn = C_bn/(C_bn'*C_bn)^0.5;                                         %保持正交化
        L   = Position_GPS(i,1);                                              %纬度
        Rx = (Ra+Position_GPS(i,3))*(1+e*sin(L)^2);                           %参考椭球子午面的法线平面内的曲率半径——东方向
        Ry = (Ra+Position_GPS(i,3))*(1-2*e+3*e*sin(L)^2);                     %参考椭球子午面内的曲率半径——北方向
        g = 9.794617509364013074*(1+0.00193185138639*sin(L)^2)/sqrt(1-0.00669437999013*sin(L)^2);  %重力加速度
        w_ie =[0; wie*cos(L); wie*sin(L)];                                    %地球自转在导航系内的分量
        w_en =[-V_y/Ry; V_x/Rx ;V_x*tan(L)/Rx];                               %载体沿地球表面运动的角速度
        
        Acc_f  = ([Ax,Ay,Az]')*g;
        Spec_f_n1 = C_bn'*Acc_f;                                                  %导航系加速度——比力方程第一项（标准输出）
        Spec_f_n2 = cross_mine(w_ie,w_en,V_velocity_0);                           %哥氏加速度——比力方程第二项
        Spec_f_n3 = [0;0;1]*g;                                                    %重力加速度修正——比力方程第三项
        Spec_Acc  = Spec_f_n1 - Spec_f_n2 + Spec_f_n3;                            %比力方程计算得到的真实加速度
    end
end

