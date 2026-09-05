clc
clear all;
m = 0.035;     %车轮的质量 The quality of the wheels
r = 0.0672/2;   %车轮的半径 Wheel radius
inertia = 0.5*m*r^2;  %车轮的转动惯量 The moment of inertia of the wheel
M = 1.000-2*m;  %车体的质量 Quality of the vehicle body   1.000就是总质量  1.000 is the total mass
L = 0.5*0.0766; %质心距底盘中心的距离The distance between the center of mass and the center of the chassis  0.0766：就是整个车体的长度只到底盘 The length of the entire vehicle body is only up to the chassis
J_centroid = (1/12)*M*(0.0766^2+0.0575^2);  %车体绕质心转动时的转动惯量The moment of inertia of the vehicle when it rotates around the center of mass  0.0766：整体高度（底板开始算）Overall height (starting from the base plate)  0.0575：底板长度一半Half the length of the base plate
d = 0.1612;   %轮距 Track width
J_Y_delata = (1/12)*M*(0.0766^2+0.0575^2);%车体绕 y 轴转动时的转动惯量 The moment of inertia when the vehicle rotates around the y-axis
g = 9.8;
Q = J_centroid*M+(J_centroid+M*L^2)*(2*m+2*inertia/r^2);
A_23 = -(M^2*L^2*g)/Q;
A_43 = M*L*g*(M+2*m+2*inertia/r^2)/Q;
B_21 = (J_centroid+M*L^2+M*L*r)/(Q*r);
B_22 = B_21;
B_41 = -(M*L/r+M+2*m+2*inertia/r^2)/Q;
B_42 = B_41;
B_61 = 1/(r*(m*d+inertia*d/r^2+2*J_Y_delata/d));
B_62 = -B_61;
A = [0 1 0 0 0 0; 0 0 A_23 0 0 0; 0 0 0 1 0 0; 0 0 A_43 0 0 0; 0 0 0 0 0 1; 0 0 0 0 0 0];
B = (inertia/r)*[0 0; B_21 B_22; 0 0; B_41 B_42; 0 0; B_61 B_62];
Tc = ctrb(A,B);
if (rank(Tc)==6)
    fprintf('此系统是可控的！\n');
    fprintf('This system is controllable!\n');
    Q = [7700 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 1600 0 0; 0 0 0 0 500 0; 0 0 0 0 0 0];
    R = [1 0; 0 1];
    K = lqr(A,B,Q,R);
end
fprintf('K:\n');
disp(K);