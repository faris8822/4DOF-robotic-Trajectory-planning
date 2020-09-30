clear
clc
close all

global SAMPLE ORDER;
ORDER = 5;   %多项式拟合的阶次


% GANG_1 = 700;
% GANG_2 = 1300;
% GANG_3 = 1300;
% GANG_4 = 1700;
% theta1=[55.22 68.91 74.84  83.00 75.42 77.10 84.27 66.71 51.50 34.42 20.12 12.94 2.48 -15.41 -30.28 -47.09];
% theta2=[7.12 -3.09 -10.83 -13.76 -30.28 -42.08 -47.29 -62.09 -69.67 -76.37 -77.49 -77.29 -78.29 -81.08 -74.84 -65.82];
% theta3=[32.67 37.93 56.31 72.67 75.90 80.64 83.54 72.09 60.72 43.72 27.79 20.40 9.34 -5.10 -16.15 -24.82];
% theta4=[21.45 18.97 0.03 -9.79 -16.93 -9.71 -16.37 -45.28 -70.99 -74.30 -72.30 -72.29 -77.02 -76.54 -77.59 -75.44];
% time = [0 1.4337 2.3165 2.6035 3.4521 3.6515 3.6749 4.1622 4.9319 8.5068 10.1059 16.1420 20.9081 24.7777 25.5319 27.0000];



% GANG_1 = 1200;
% GANG_2 = 1300;
% GANG_3 = 1300;
% GANG_4 = 1300;
% theta1=[33.80 33.80 33.80 39.45 48.33 52.56 52.56 52.56 52.56 52.56 52.56 52.56 45.67 36.21 16.60 -27.89 -51.18];
% theta2=[2.82 2.82 2.82 -5.74 -19.75 -28.48 -39.21 -42.00 -47.62 -52.80 -54.68 -52.44 -60.02 -65.04 -55.19 -28.31 -51.47];
% theta3=[34.73 41.28 48.17 54.08 68.47 86.18 84.17 84.16 65.30 44.25 23.20 4.14 -7.87 -24.34 -53.09 -68.15 -54.10];
% theta4=[34.58 23.41 12.63 5.29 1.39 0.17 -21.41 -34.95 -60.50 -75.14 -84.17 -86.31 -82.92 -81.90 -81.90 -82.38 -54.48];
% time = [0 0.6860 1.2960 1.4538 1.8022 2.4656 2.9301 3.4932 6.6921 10.5527 13.3559 14.6580 17.4884 19.4233 22.1112 25.1506 27];

GANG_1 = 910;
GANG_2 = 1550;
GANG_3 = 1550;
GANG_4 = 1330;
theta1=[5.43 5.43 5.43 5.43 5.43 5.43 5.43 5.43 5.43 -15.43 -25];
theta2=[53.56 63.56 66.76 103.56 93.56 73.56 56.89 23.56 -20.44 -41.3 -50];
theta3=[6.16 6.16 6.16 6.16 -3.84 -23.84 -40.51 -73.84 -80.84 -71.7 -70];
theta4=[33.69 6.16 6.16 -19.408 -61.37 -81.37 -98.04 -93.9 -100.9 -72.29 -51.12];
time = [0 1 1.6 2.2 5 11 14 18 23.8 26 27];




%======================================================================================================================================
%角度转成弧度制，画出未拟合的轨迹


outline_x=0.1*[-46300  -46300 -45300 -44197.08 -44410.27 -35421.79 -35121.34 -16100 -16100 -11600 -11600 -6450 -6450 6350 6350 13550 13550 22000 22000 25750 25750 22000 22000 13550 13550 6350 6350 950]; %-950 -950 -6450 -6450 -1610 -1610];
outline_y=0.1*[-1008.1  -28678.1 -28678.1 -28918.81 -29895.82 -31857.42 -30880.23 -35050 -34550 -34550 -31270 -31270 -32040 -32040 -35175 -35175 -51729.5 -51729.5 -45150 -45150 -42270 -42270 21189.5 21189.5 4635 4635 1500 1500]; %-950 -950 -6450 -6450 -1610 -1610];
hold on 
plot(outline_x,outline_y)


[temp1,temp2] = size(theta1);
SAMPLE = temp2;
clear temp1  temp2
theta1 = 2*pi*theta1/360;
theta2 = 2*pi*theta2/360;
theta3 = 2*pi*theta3/360;
theta4 = 2*pi*theta4/360;
x_1 = GANG_1*sin(theta1);
y_1 = -(GANG_1*cos(theta1));
x_2 = GANG_1*sin(theta1)+GANG_2*sin(theta2);
y_2 = -(GANG_1*cos(theta1)+GANG_2*cos(theta2));
x_3 = GANG_1*sin(theta1)+GANG_2*sin(theta2)+GANG_3*sin(theta3);
y_3 = -(GANG_1*cos(theta1)+GANG_2*cos(theta2)+GANG_3*cos(theta3));
x_4 = GANG_1*sin(theta1)+GANG_2*sin(theta2)+GANG_3*sin(theta3)+GANG_4*sin(theta4);
y_4 = -(GANG_1*cos(theta1)+GANG_2*cos(theta2)+GANG_3*cos(theta3)+GANG_4*cos(theta4));
plot(x_1,y_1);
plot(x_2,y_2);
plot(x_3,y_3);
plot(x_4,y_4);
hold off

%==================================================
%多项式拟合
theta4 = theta4 - theta3;
theta3 = theta3 - theta2;
theta2 = theta2 - theta1;

a1 = polyfit(time,theta1,ORDER);
a2 = polyfit(time,theta2,ORDER);
a3 = polyfit(time,theta3,ORDER);
a4 = polyfit(time,theta4,ORDER);
x = 0:0.5: time(SAMPLE);
theta1_pf = polyval(a1,x);
theta2_pf  = polyval(a2,x);
theta3_pf  = polyval(a3,x);
theta4_pf  = polyval(a4,x);



%===============================
%角度变换
theta1_pf_v2 = theta1_pf;
theta2_pf_v2 = theta2_pf;
theta3_pf_v2 = theta3_pf;
theta4_pf_v2 = theta4_pf;
theta2_pf_v2 = theta2_pf_v2 + theta1_pf_v2;
theta3_pf_v2 = theta3_pf_v2 + theta2_pf_v2;
theta4_pf_v2 = theta4_pf_v2 + theta3_pf_v2;

%==========================================
%拟合后的多项式
x_1_pf = GANG_1*sin(theta1_pf_v2);
y_1_pf = -(GANG_1*cos(theta1_pf_v2));
x_2_pf = GANG_1*sin(theta1_pf_v2)+GANG_2*sin(theta2_pf_v2);
y_2_pf = -(GANG_1*cos(theta1_pf_v2)+GANG_2*cos(theta2_pf_v2));
x_3_pf = GANG_1*sin(theta1_pf_v2)+GANG_2*sin(theta2_pf_v2)+GANG_3*sin(theta3_pf_v2);
y_3_pf = -(GANG_1*cos(theta1_pf_v2)+GANG_2*cos(theta2_pf_v2)+GANG_3*cos(theta3_pf_v2));
x_4_pf = GANG_1*sin(theta1_pf_v2)+GANG_2*sin(theta2_pf_v2)+GANG_3*sin(theta3_pf_v2)+GANG_4*sin(theta4_pf_v2);
y_4_pf = -(GANG_1*cos(theta1_pf_v2)+GANG_2*cos(theta2_pf_v2)+GANG_3*cos(theta3_pf_v2)+GANG_4*cos(theta4_pf_v2));

% x_1_pf = GANG_1*sin(theta1_pf);
% y_1_pf = -(GANG_1*cos(theta1_pf));
% x_2_pf = GANG_1*sin(theta1_pf)+GANG_2*sin(theta2_pf);
% y_2_pf = -(GANG_1*cos(theta1_pf)+GANG_2*cos(theta2_pf));
% x_3_pf = GANG_1*sin(theta1_pf)+GANG_2*sin(theta2_pf)+GANG_3*sin(theta3_pf);
% y_3_pf = -(GANG_1*cos(theta1_pf)+GANG_2*cos(theta2_pf)+GANG_3*cos(theta3_pf));
% x_4_pf = GANG_1*sin(theta1_pf)+GANG_2*sin(theta2_pf)+GANG_3*sin(theta3_pf)+GANG_4*sin(theta4_pf);
% y_4_pf = -(GANG_1*cos(theta1_pf)+GANG_2*cos(theta2_pf)+GANG_3*cos(theta3_pf)+GANG_4*cos(theta4_pf));


figure(2);
hold on
plot(outline_x,outline_y)
plot(x_1_pf,y_1_pf);
plot(x_2_pf,y_2_pf);
plot(x_3_pf,y_3_pf);
plot(x_4_pf,y_4_pf);
hold off
%=============================================
%角度变换
% theta4 = theta4 - theta3;
% theta3 = theta3 - theta2;
% theta2 = theta2 - theta1;
%theta1 =  theta1;
% theta4_pf = theta4_pf - theta3_pf;
% theta3_pf = theta3_pf - theta2_pf;
% theta2_pf = theta2_pf - theta1_pf;
%theta1_pf = theta1_pf;
%=============================================



L(1) = Link([0 0 GANG_1*0.001 0]);
L(2) = Link([0 0 GANG_2*0.001 0]);
L(3) = Link([0 0 GANG_3*0.001 0]);
L(4) = Link([0 0 GANG_4*0.001 0]);
triDOF_robot = SerialLink(L);
theta = [theta1_pf;theta2_pf;theta3_pf;theta4_pf]';

figure(3);
view([0 0 1])
triDOF_robot.plot(theta);


