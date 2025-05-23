clc
clear;

theta1min = -180; theta1max = 180;
theta2min = -180; theta2max = 180;
theta3min = -180; theta3max = 180;
theta4min = -180; theta4max = 180;
theta5min = -180; theta5max = 180;
theta6min = -180; theta6max = 180;

n=30000;
x = zeros(n,1);
y = zeros(n,1 );
z = zeros(n,1);
for i = 1:n
    theta1 = theta1min*(pi/180) + (theta1max - theta1min)*(pi/180)*rand;
    theta2 = theta2min*(pi/180) + (theta2max - theta2min)*(pi/180)*rand;
    theta3 = theta3min*(pi/180) + (theta3max - theta3min)*(pi/180)*rand;
    theta4 = theta4min*(pi/180) + (theta4max - theta4min)*(pi/180)*rand;
    theta5 = theta5min*(pi/180) + (theta5max - theta5min)*(pi/180)*rand;
    theta6 = theta6min*(pi/180) + (theta6max - theta6min)*(pi/180)*rand;
    Tws = modelRobot([theta1, theta2, theta3, theta4, theta5, theta6]);
    x(i) = Tws(1,1);
    y(i) = Tws(2,1);
    z(i) = Tws(3,1);
end

figure('color', [1 1 1]);
plot3(x, y, z, 'b.', 'MarkerSize', 0.5);
hold on;
xlabel('x轴(millimeter)', 'color', 'r', 'fontsize', 15);
ylabel('y轴(millimeter)', 'color', 'r', 'fontsize', 15);
zlabel('z轴(millimeter)', 'color', 'r', 'fontsize', 15);
grid on;


% 求齐次变换矩阵 (standard DH)
function T = DHTrans(theta, d, a, alpha)
T= [cos(theta)     -sin(theta)*cos(alpha)     sin(theta)*sin(alpha)      a*cos(theta);
    sin(theta)      cos(theta)*cos(alpha)     -cos(theta)*sin(alpha)     a*sin(theta);
    0               sin(alpha)                 cos(alpha)                   d;
    0                      0                      0                         1];
end

function Rxyz=RotMat_AxisAngle(R)
theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
r = 1/2/sin(theta)*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
Rxyz=theta*r;
end


% 输入机器人q1,q2,q3,q4,q5,q6的角度
% 输出末端位姿的轴线表示
function position = modelRobot(theta)

th(1) = theta(1); d(1) = 0.1065 ; a(1) =   0    ; alp(1) =  pi/2;
th(2) = theta(2); d(2) = 0      ; a(2) =  -0.408; alp(2) =  0   ;
th(3) = theta(3); d(3) = 0      ; a(3) =  -0.382; alp(3) =  0    ;
th(4) = theta(4); d(4) = 0.1109 ; a(4) =   0    ; alp(4) =  pi/2 ;
th(5) = theta(5); d(5) = 0.1109 ; a(5) =   0    ; alp(5) = -pi/2;
th(6) = theta(6); d(6) = 0.08409; a(6) =   0    ; alp(6) =  0   ;

T01 = DHTrans(th(1), d(1), a(1), alp(1));
T12 = DHTrans(th(2), d(2), a(2), alp(2));
T23 = DHTrans(th(3), d(3), a(3), alp(3));
T34 = DHTrans(th(4), d(4), a(4), alp(4));
T45 = DHTrans(th(5), d(5), a(5), alp(5));
T56 = DHTrans(th(6), d(6), a(6), alp(6));

T06=T01*T12*T23*T34*T45*T56;
axis = RotMat_AxisAngle(T06)/pi*180;
position = [T06(1:3,4:4),axis];
end



