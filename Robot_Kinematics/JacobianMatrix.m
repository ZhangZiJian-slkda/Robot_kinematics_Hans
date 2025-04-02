clc;
clear;
%format long g; %输出结果为小数
syms d1 d2 d3  a2 a3 d4 a4 d5 d6_2 a7 th1 th2 th3 th4 th5 th6 d7 th7 a7 e3 e2 
syms alp7 th8 d8 a8 alp8  th10 d10 a10 alp10 d6_2 a9 d6 d9
 
th(10) = th10;   d(10) = d10;  a(10) = a10; alp(10) = alp10;
 
th(1) = th1; d(1) = d1; a(1) = 0;  alp(1) = 0;
th(2) = th2+pi/2; d(2) = 0;  a(2) = 0;  alp(2) = pi/2;      
th(3) = th3; d(3) = 0;  a(3) = a3; alp(3) = 0;
th(4) = th4-pi/2; d(4) = d4; a(4) = a4; alp(4) = 0;            
th(5) = th5; d(5) = d5; a(5) = 0;  alp(5) = -pi/2;
th(6) = th6; d(6) = d6; a(6) = 0;  alp(6) = pi/2;      
 
T01 = connectingRodTransfer([alp(1), a(1), d(1), th(1)],0);
T12 = connectingRodTransfer([alp(2), a(2), d(2), th(2)],0);
T23 = connectingRodTransfer([alp(3), a(3), d(3), th(3)],0);
T34 = connectingRodTransfer([alp(4), a(4), d(4), th(4)],0);
T45 = connectingRodTransfer([alp(5), a(5), d(5), th(5)],0);
T56 = connectingRodTransfer([alp(6), a(6), d(6), th(6)],0);%
T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
T06 = T01*T12*T23*T34*T45*T56;
P01 = [T01(1,4);T01(2,4);T01(3,4)];
P02 = [T02(1,4);T02(2,4);T02(3,4)];
P03 = [T03(1,4);T03(2,4);T03(3,4)];
P04 = [T04(1,4);T04(2,4);T04(3,4)];
P05 = [T05(1,4);T05(2,4);T05(3,4)];
P06 = [T06(1,4);T06(2,4);T06(3,4)];
 
z1 = T01(1:3,3);
z2 = T02(1:3,3);
z3 = T03(1:3,3);
z4 = T04(1:3,3);
z5 = T05(1:3,3);
z6 = T06(1:3,3);
j1 = [cross(z1,P06-P01);z1];
j2 = [cross(z2,P06-P02);z2];
j3 = [cross(z3,P06-P03);z3];
j4 = [cross(z4,P06-P04);z4];
j5 = [cross(z5,P06-P05);z5];
j6 = [cross(z6,P06-P06);z6];
jacobian0 = [j1,j2,j3,j4,j5,j6];
d = det(jacobian0);
simplify(d)
function TArr = connectingRodTransfer(dh,theta,inv)
if nargin == 2
    inv=0;
end
ct = cos(dh(4)+theta);
st = sin(dh(4)+theta);
ca = cos(dh(1));
sa = sin(dh(1));
a = dh(2);
d = dh(3);
TArr = [    ct      -st     0       a;
            st*ca   ct*ca   -sa     -sa*d;
            st*sa   ct*sa   ca      ca*d;
            0       0       0       1   ];
if inv
    TArr=[TArr(1:3,1:3)' -TArr(1:3,1:3)'*TArr(1:3,4);0 0 0 1];
end
end