clc;
clear;
close all;

global SAMPLE STEP TIME;
STEP = 0.01; %积分的采样点数，影响后面积分的准确性
TIME = 27;
%角度
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%对应杆长 910-1550-1550-1330
% GANG_1 = 910;
% GANG_2 = 1550;
% GANG_3 = 1550;
% GANG_4 = 1330;
% theta1=[5.43 5.43 5.43 5.43 5.43 5.43 5.43 5.43 5.43 -15.43 -25];
% theta2=[53.56 63.56 66.76 103.56 93.56 73.56 56.89 23.56 -20.44 -41.3 -50];
% theta3=[6.16 6.16 6.16 6.16 -3.84 -23.84 -40.51 -73.84 -80.84 -71.7 -70];
% theta4=[33.69 6.16 6.16 -19.408 -61.37 -81.37 -98.04 -93.9 -100.9 -72.29 -51.12];

% GANG_1 = 700;
% GANG_2 = 1600;
% GANG_3 = 1600;
% GANG_4 = 1300;
% theta1=[27.04 85.87 91.46 91.11 85.02 56.17 17.07 -25.54 -41.97 -49.31 -51.91];
% theta2=[24.62 36.36 37.48 37.67 40.88 28.83 8.76 -25.51 -41.92 -49.25 -51.85];
% theta3=[29.74 2.58 10.26 14.64 3.22 -63.36 -89.96 -67.36 -58.62 -55.41 -54.34];
% theta4=[19.51 1.69 0 -87.77 -80.3 -107.04 -119.93 -81.94 -67.24 -61.84 -60.04];


% GANG_1 = 900;
% GANG_2 = 1600;
% GANG_3 = 1600;
% GANG_4 = 1100;
% theta1=[38.26 51.53 58.84 61.68 65 45.37 19.98 -12.85 -26.68 -43.34 -33.7];
% theta2=[6.65 -6.34 -12.68 -21.19 -55 -52.3 -61.08 -63.37 -62.3 -57.96  -62.5];
% theta3=[39.88 48.23 52.91 70 67.01 55.98 42.96 22.88 13.96 1.45  -28.72];
% theta4=[26.2 10.65 1.06 -10 -40 -75.23 -90 -99.87 -103.1 -103.65  -83.71];


% GANG_1 = 1200;
% GANG_2 = 1300;
% GANG_3 = 1300;
% GANG_4 = 1300;
% theta1=[33.80 33.80 33.80 39.45 48.33 52.56 52.56 52.56 52.56 52.56 52.56 52.56 45.67 36.21 16.60 -27.89 -51.18];
% theta2=[2.82 2.82 2.82 -5.74 -19.75 -28.48 -39.21 -42.00 -47.62 -52.80 -54.68 -52.44 -60.02 -65.04 -55.19 -28.31 -51.47];
% theta3=[34.73 41.28 48.17 54.08 68.47 86.18 84.17 84.16 65.30 44.25 23.20 4.14 -7.87 -24.34 -53.09 -68.15 -54.10];
% theta4=[34.58 23.41 12.63 5.29 1.39 0.17 -21.41 -34.95 -60.50 -75.14 -84.17 -86.31 -82.92 -81.90 -81.90 -82.38 -54.48];



GANG_1 = 700;
GANG_2 = 1300;
GANG_3 = 1300;
GANG_4 = 1700;
theta1=[55.22 68.91 74.84  83.00 75.42 77.10 84.27 66.71 51.50 34.42 20.12 12.94 2.48 -15.41 -30.28 -47.09];
theta2=[7.12 -3.09 -10.83 -13.76 -30.28 -42.08 -47.29 -62.09 -69.67 -76.37 -77.49 -77.29 -78.29 -81.08 -74.84 -65.82];
theta3=[32.67 37.93 56.31 72.67 75.90 80.64 83.54 72.09 60.72 43.72 27.79 20.40 9.34 -5.10 -16.15 -24.82];
theta4=[21.45 18.97 0.03 -9.79 -16.93 -9.71 -16.37 -45.28 -70.99 -74.30 -72.30 -72.29 -77.02 -76.54 -77.59 -75.44];



%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

[temp1,temp2] = size(theta1);
SAMPLE = temp2;
time = [1:SAMPLE-1 TIME];
clear temp1  temp2

%第一种分配时间的方案  四个关节的变化都加上
RUN_TIME = time(SAMPLE); %运动得时间，为time数组的最后一个数。
time_new = [0];
all_deta = 0;
for i = 1:SAMPLE-1
    deta1 = abs(theta1(i+1)-theta1(i));
    deta2 = abs(theta2(i+1)-theta2(i));
    deta3 = abs(theta3(i+1)-theta3(i));
    deta4 = abs(theta4(i+1)-theta4(i));
    sum_deta(i) = deta1+deta2+deta3+deta4; 
    all_deta = all_deta+sum_deta(i);
end

for i = 1:SAMPLE-1
    time_new = [time_new time_new(i)+sum_deta(i)/all_deta*RUN_TIME];
end
time = time_new;
time_1 = time_new;
figure(1);
run yake_v5_NO1.m
%run yake_v5_NO1.m
fprintf('\n==========================================================================\n');
fprintf('                   四个关节角度的变化都加上(%f s)                              \n',time_1(SAMPLE));
Pmax1=max(P(1,:));
Pmax2=max(P(2,:));
Pmax3=max(P(3,:));
Pmax4=max(P(4,:));
fprintf('关节1=%f  |  关节2=%f  |  关节3=%f  |  关节4=%f',Pmax1,Pmax2,Pmax3,Pmax4);
fprintf('\n==========================================================================\n');
time_1

%==============================================================================
%第二种分配时间的方案 取每组四个关节的最大值。
RUN_TIME = time(SAMPLE); %运动得时间，为time数组的最后一个数。
time_new = [0];
all_deta = 0;
for i = 1:SAMPLE-1
    deta1 = abs(theta1(i+1)-theta1(i));
    deta2 = abs(theta2(i+1)-theta2(i));
    deta3 = abs(theta3(i+1)-theta3(i));
    deta4 = abs(theta4(i+1)-theta4(i));
    sum_deta(i) = max([deta1 deta2 deta3 deta4]); 
    all_deta = all_deta+sum_deta(i);
end

for i = 1:SAMPLE-1
    time_new = [time_new time_new(i)+sum_deta(i)/all_deta*RUN_TIME];
end
time = time_new;
time_2 = time_new;
figure(2);
run yake_v5_NO1.m
fprintf('\n==========================================================================\n');
fprintf('                   取每组四个关节的最大值(%f s)                              \n',time_2(SAMPLE));
Pmax1=max(P(1,:));
Pmax2=max(P(2,:));
Pmax3=max(P(3,:));
Pmax4=max(P(4,:));
fprintf('关节1=%f  |  关节2=%f  |  关节3=%f  |  关节4=%f',Pmax1,Pmax2,Pmax3,Pmax4);fprintf('\n==========================================================================\n');
time_2

%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%第三种改变时间的方案  （四个关节角度的变化*关节的稳定性）之和
RUN_TIME = time(SAMPLE)-2;
time_new = [0];
all_deta = 0;
for i = 1:SAMPLE-1
    deta1 = abs(theta1(i+1)-theta1(i));
    deta2 = abs(theta2(i+1)-theta2(i));
    deta3 = abs(theta3(i+1)-theta3(i));
    deta4 = abs(theta4(i+1)-theta4(i));
    sum_deta(i) = deta1+deta2+deta3+deta4; 
    all_deta = all_deta+sum_deta(i);
end

for i = 1:SAMPLE-1
    time_new = [time_new time_new(i)+sum_deta(i)/all_deta*RUN_TIME];
end
time = time_new;
figure(5);
run yake_v5_NO1.m
h = figure(5);
close(h);
[temp1,temp2]= size(P);
time = round((time*temp2)/RUN_TIME);       %temp2为平稳性P的采样点数
clear temp1 temp2;
%time(i+1)到time(i)时间段的积分
for i = 1:SAMPLE-1
      w1(i) = sum(P(1,time(i)+1:time(i+1)))/100000;
      w2(i) = sum(P(2,time(i)+1:time(i+1)))/100000;
      w3(i) = sum(P(3,time(i)+1:time(i+1)))/100000;
      w4(i) = sum(P(4,time(i)+1:time(i+1)))/100000;
end
clear temp;
time_new = [0];
all_deta = 0;
for i = 1:SAMPLE-1
    deta1 = abs(theta1(i+1)-theta1(i));
    deta2 = abs(theta2(i+1)-theta2(i));
    deta3 = abs(theta3(i+1)-theta3(i));
    deta4 = abs(theta4(i+1)-theta4(i));
    sum_deta(i) =  w1(i)*deta1+w2(i)*deta2+w3(i)*deta3+w4(i)*deta4; 
    all_deta = all_deta+sum_deta(i);
end

for i = 1:SAMPLE-1
    time_new = [time_new time_new(i)+sum_deta(i)/all_deta*RUN_TIME];
end

%在时间的始末个加上一秒
time_new(2)=time_new(2)+0.5;
for i = 3:SAMPLE-2
    time_new(i)= time_new(i)+1;
end
time_new(SAMPLE-1) = time_new(SAMPLE-1)+1.5;
time_new(SAMPLE) = time_new(SAMPLE)+2;
time = time_new;   
time_3 = time_new;
figure(3);
run yake_v5_NO1.m
fprintf('\n==========================================================================\n');
fprintf('              （四个关节角度的变化*关节的稳定性）之和（%f s）                       \n',time_3(SAMPLE));
Pmax1_NO3=max(P(1,:));
Pmax2_NO3=max(P(2,:));
Pmax3_NO3=max(P(3,:));
Pmax4_NO3=max(P(4,:));
fprintf('关节1=%f  |  关节2=%f  |  关节3=%f  |  关节4=%f',Pmax1_NO3,Pmax2_NO3,Pmax3_NO3,Pmax4_NO3);
fprintf('\n==========================================================================\n');
time_3


%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%第四种改变时间的方案  四个关节角度的变化取最大的那个*关节的稳定性
RUN_TIME  = time(SAMPLE)-2;
time_new = [0];
all_deta = 0;
for i = 1:SAMPLE-1
    deta1 = abs(theta1(i+1)-theta1(i));
    deta2 = abs(theta2(i+1)-theta2(i));
    deta3 = abs(theta3(i+1)-theta3(i));
    deta4 = abs(theta4(i+1)-theta4(i));
    sum_deta(i) = max([deta1 deta2 deta3 deta4]); 
    all_deta = all_deta+sum_deta(i);
end

for i = 1:SAMPLE-1
    time_new = [time_new time_new(i)+sum_deta(i)/all_deta*RUN_TIME];
end
time = time_new;
figure(5);
%run yake_v6_NO1.m
run yake_v5_NO1.m
h = figure(5);
close(h);
[temp1,temp2]= size(P);
time = round((time*temp2)/RUN_TIME);       %temp2为平稳性P的采样点数
clear temp1 temp2;
%time(i+1)到time(i)时间段的积分
for i = 1:SAMPLE-1
%       temp1 = int8(time(i));
%       temp2 = int8(time(i+1));
      w1(i) = sum(P(1,time(i)+1:time(i+1)))/100000;
      w2(i) = sum(P(2,time(i)+1:time(i+1)))/100000;
      w3(i) = sum(P(3,time(i)+1:time(i+1)))/100000;
      w4(i) = sum(P(4,time(i)+1:time(i+1)))/100000;
end
clear temp;
% w1 = mapminmax(w1,0,1);
time_new = [0];
all_deta = 0;
for i = 1:SAMPLE-1
%     deta1 = abs(theta1(i+1)-theta1(i));
%     deta2 = abs(theta2(i+1)-theta2(i));
%     deta3 = abs(theta3(i+1)-theta3(i));
%     deta4 = abs(theta4(i+1)-theta4(i));
    sum_deta(i) =  max([w1(i)*deta1 w2(i)*deta2 w3(i)*deta3 w4(i)*deta4]); 
    all_deta = all_deta+sum_deta(i);
end

for i = 1:SAMPLE-1
    time_new = [time_new time_new(i)+sum_deta(i)/all_deta*RUN_TIME];
end

%在时间的始末个加上一秒
time_new(2)=time_new(2)+0.5;
for i = 3:SAMPLE-2
    time_new(i)= time_new(i)+1;
end
time_new(SAMPLE-1) = time_new(SAMPLE-1)+1.5;
time_new(SAMPLE) = time_new(SAMPLE)+2;
time = time_new;   
time_4 = time_new;
figure(4);
run yake_v5_NO1.m
fprintf('\n==========================================================================\n');
fprintf('              四个关节角度的变化取最大的那个*关节的稳定性（%f s）                     \n',time_4(SAMPLE));
Pmax1_NO3=max(P(1,:));
Pmax2_NO3=max(P(2,:));
Pmax3_NO3=max(P(3,:));
Pmax4_NO3=max(P(4,:));
fprintf('关节1=%f  |  关节2=%f  |  关节3=%f  |  关节4=%f',Pmax1_NO3,Pmax2_NO3,Pmax3_NO3,Pmax4_NO3);
fprintf('\n==========================================================================\n');
time_4







