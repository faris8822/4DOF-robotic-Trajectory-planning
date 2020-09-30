clear
clc
close all
global SAMPLE STEP ORDER;
ORDER = 5;   %多项式拟合的阶次
STEP = 0.01; %积分的采样点数，影响后面积分的准确性


% 
% GANG_1 = 910;
% GANG_2 = 1550;
% GANG_3 = 1550;
% GANG_4 = 1330;
% theta1=[5.43 5.43 5.43 5.43 5.43 5.43 5.43 5.43 5.43 -15.43 -25];
% theta2=[53.56 63.56 66.76 103.56 93.56 73.56 56.89 23.56 -20.44 -41.3 -50];
% theta3=[6.16 6.16 6.16 6.16 -3.84 -23.84 -40.51 -73.84 -80.84 -71.7 -70];
% theta4=[33.69 6.16 6.16 -19.408 -61.37 -81.37 -98.04 -93.9 -100.9 -72.29 -51.12];
% time = [0 1 1.6 2.2 5 11 14 18 23.8 26 27];


GANG_1 = 1200;
GANG_2 = 1300;
GANG_3 = 1300;
GANG_4 = 1300;
theta1=[33.80 33.80 33.80 39.45 48.33 52.56 52.56 52.56 52.56 52.56 52.56 52.56 45.67 36.21 16.60 -27.89 -51.18];
theta2=[2.82 2.82 2.82 -5.74 -19.75 -28.48 -39.21 -42.00 -47.62 -52.80 -54.68 -52.44 -60.02 -65.04 -55.19 -28.31 -51.47];
theta3=[34.73 41.28 48.17 54.08 68.47 86.18 84.17 84.16 65.30 44.25 23.20 4.14 -7.87 -24.34 -53.09 -68.15 -54.10];
theta4=[34.58 23.41 12.63 5.29 1.39 0.17 -21.41 -34.95 -60.50 -75.14 -84.17 -86.31 -82.92 -81.90 -81.90 -82.38 -54.48];
time = [0 0.6860 1.2960 1.4538 1.8022 2.4656 2.9301 3.4932 6.6921 10.5527 13.3559 14.6580 17.4884 19.4233 22.1112 25.1506 27];
 



%===========================================================
% theta4 = theta4 - theta3;
% theta3 = theta3 - theta2;
% theta2 = theta2 - theta1;



%theta1 = 90 - theta1;
[temp1,temp2] = size(theta1);
SAMPLE = temp2;
clear temp1  temp2



a1 = polyfit(time,theta1,ORDER);
a2 = polyfit(time,theta2,ORDER);
a3 = polyfit(time,theta3,ORDER);
a4 = polyfit(time,theta4,ORDER);
x = 0:0.01: time(SAMPLE);
y1 = polyval(a1,x);
y2 = polyval(a2,x);
y3 = polyval(a3,x);
y4 = polyval(a4,x);



subplot(3,1,1);
title('各关节变化曲线');
xlabel('时间/s');
ylabel('角度( °)');
grid on;
hold on
plot(x,y1,time,theta1);
plot(x,y2,time,theta2);
plot(x,y3,time,theta3);
plot(x,y4,time,theta4);
legend('theta1','theta2','theta3','theta4');
hold off



theta_1 = 2*pi*y1/360;
theta_2 = 2*pi*y2/360;
theta_3 = 2*pi*y3/360;
theta_4 = 2*pi*y4/360;

L(1) = Link([0 0 GANG_1*0.001 0]);
L(2) = Link([0 0 GANG_2*0.001 0]);
L(3) = Link([0 0 GANG_3*0.001 0]);
L(4) = Link([0 0 GANG_4*0.001 0]);

robot_4DOF = SerialLink(L);
t = [];
[temp,sample2] = size(theta_1); 
clear temp
for i = 1:1:sample2
    % theta= [0 0 0 0];
    J = robot_4DOF.jacob0([theta_1(i) theta_2(i) theta_3(i) theta_4(i)]);
    % robot_4DOF.jacobe(theta1)
    F = [2500 0 0 0 0 0];
    t = [t J'*F'];
end
%x = 1/sample2:time(SAMPLE)/sample2:time(SAMPLE);

subplot(3,1,2);
%figure(2)
title('各关节的力矩');
xlabel('时间/s');
ylabel('力矩(（N·m）)');
grid on;
hold on
plot(x,t(1,:));
plot(x,t(2,:));
plot(x,t(3,:));
plot(x,t(4,:));
legend('关节1','关节2','关节3','关节4');
hold off

% 

for j=1:sample2-1
    P(1, j)=(t(1, j+1)-t(1,j))/STEP;
    P(2, j)=(t(2, j+1)-t(2,j))/STEP;
    P(3, j)=(t(3, j+1)-t(3,j))/STEP;
    P(4, j)=(t(4, j+1)-t(4,j))/STEP;
end


%画出平稳性的变化趋势
[temp1,temp2] = size(P);
%figure(3)
subplot(3,1,3);
%set(gca, 'Units', 'normalized', 'Position', [0.505 0.505 0.495 0.495])
title('各关节平稳性（斜率）');
xlabel('时间/s');
ylabel('N·m/S');
grid on;
hold on
plot(x(1:temp2),P(1,:));
plot(x(1:temp2),P(2,:));
plot(x(1:temp2),P(3,:));
plot(x(1:temp2),P(4,:));
legend('关节1','关节2','关节3','关节4');
hold off
P=abs(P);
%求解平稳性
fprintf('\n==========================================================================\n');
fprintf('             (%.0f 阶多项式拟合)各关节平稳性(%f s)                           \n',ORDER,time(SAMPLE));
Pmax1=max(P(1,:));
Pmax2=max(P(2,:));
Pmax3=max(P(3,:));
Pmax4=max(P(4,:));
fprintf('关节1=%f  |  关节2=%f  |  关节3=%f  |  关节4=%f',Pmax1,Pmax2,Pmax3,Pmax4);
fprintf('\n==========================================================================\n');
%求解平稳性最大值


% fid=fopen('theta1.txt','w');
% for i = 1:50:sample2
%     fprintf(fid,'%4f %4f\r\n',x(i),90-y1(i));
% end
% fclose(fid);
% 
% fid=fopen('theta2.txt','w');
% for i = 1:50:sample2
%     fprintf(fid,'%4f %4f\r\n',x(i),y1(i)-y2(i));
% end
% fclose(fid);
% 
% fid=fopen('theta3.txt','w');
% for i = 1:50:sample2
%     fprintf(fid,'%4f %4f\r\n',x(i),y2(i)-y3(i));
% end
% fclose(fid);
% 
% fid=fopen('theta4.txt','w');
% for i = 1:50:sample2
%     fprintf(fid,'%4f %4f\r\n',x(i),y3(i)-y4(i));
% end
% fclose(fid);














