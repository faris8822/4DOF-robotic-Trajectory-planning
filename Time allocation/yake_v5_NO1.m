
%===================================================================================================
%theta1 ���εĹ켣�滮�����ɵ�zhi_1Ϊ4xi�ľ���ÿ�е�4��Ԫ�ر�ʾ��켣���̵��ĸ�������
for i = 1:1:SAMPLE-1  %һ��ʮһ�������㣬���Թ�����10�����ߣ���Ҫѭ��10�Ρ�
    val = [time(i)*time(i)*time(i) time(i)*time(i) time(i) 1;...
           time(i+1)*time(i+1)*time(i+1) time(i+1)*time(i+1) time(i+1) 1;...
           3*time(i)*time(i) 2*time(i) 1 0;...
           3*time(i+1)*time(i+1) 2*time(i+1) 1 0];
    con = [theta1(i) theta1(i+1) 0 0]';
    zhi_1(:,i) =inv(val)*con;  %���þ�����ⷽ���顣
end
%theat2
for i = 1:1:SAMPLE-1
    val = [time(i)*time(i)*time(i) time(i)*time(i) time(i) 1;...
           time(i+1)*time(i+1)*time(i+1) time(i+1)*time(i+1) time(i+1) 1;...
           3*time(i)*time(i) 2*time(i) 1 0;...
           3*time(i+1)*time(i+1) 2*time(i+1) 1 0];
    con = [theta2(i) theta2(i+1) 0 0]';
    zhi_2(:,i) =inv(val)*con;  
end
%theta3
for i = 1:1:SAMPLE-1
    val = [time(i)*time(i)*time(i) time(i)*time(i) time(i) 1;...
           time(i+1)*time(i+1)*time(i+1) time(i+1)*time(i+1) time(i+1) 1;...
           3*time(i)*time(i) 2*time(i) 1 0;...
           3*time(i+1)*time(i+1) 2*time(i+1) 1 0];
    con = [theta3(i) theta3(i+1) 0 0]';
    zhi_3(:,i) =inv(val)*con;  
end
%theta4
for i = 1:1:SAMPLE-1

    val = [time(i)*time(i)*time(i) time(i)*time(i) time(i) 1;...
           time(i+1)*time(i+1)*time(i+1) time(i+1)*time(i+1) time(i+1) 1;...
           3*time(i)*time(i) 2*time(i) 1 0;...
           3*time(i+1)*time(i+1) 2*time(i+1) 1 0];
    con = [theta4(i) theta4(i+1) 0 0]';
    zhi_4(:,i) = inv(val)*con;  
end

%===================================================================================
%ͨ��֮ǰ�Ķ���ʽ��������д������ʽ�ķ��̣�Ȼ����STEP�Ĳ������в��������������ɵ�ͼ��
theta1_fig_1 = [];
t_x= [];
theta1_fig_2 = [];
%t_x_2= [];
theta1_fig_3 = [];
%t_x_3= [];
theta1_fig_4 = [];
%t_x_4= [];
%t_x = time(1):STEP:time(SAMPLE);
for  i = 1:1:SAMPLE-1 
     t = round(time(i)*100)/100:STEP:round(time(i+1)*100)/100;
     t_x = [t_x t];
    theta1_fig_1 = [theta1_fig_1 zhi_1(1,i).*t.*t.*t+zhi_1(2,i).*t.*t+zhi_1(3,i).*t+zhi_1(4,i)];  
end
for  i = 1:1:SAMPLE-1
    t = round(time(i)*100)/100:STEP:round(time(i+1)*100)/100;
    %t_x_2 = [t_x_2 t];
    theta1_fig_2 = [theta1_fig_2 zhi_2(1,i).*t.*t.*t+zhi_2(2,i).*t.*t+zhi_2(3,i).*t+zhi_2(4,i)];  
end
for  i = 1:1:SAMPLE-1
    t = round(time(i)*100)/100:STEP:round(time(i+1)*100)/100;
    %t_x_3 = [t_x_3 t];
    theta1_fig_3 = [theta1_fig_3 zhi_3(1,i).*t.*t.*t+zhi_3(2,i).*t.*t+zhi_3(3,i).*t+zhi_3(4,i)];  
end
for  i = 1:1:SAMPLE-1
    t = round(time(i)*100)/100:STEP:round(time(i+1)*100)/100;
    %t_x_4 = [t_x_4 t];
    theta1_fig_4 = [theta1_fig_4 zhi_4(1,i).*t.*t.*t+zhi_4(2,i).*t.*t+zhi_4(3,i).*t+zhi_4(4,i)];  
end




subplot(3,1,1);
title('���ؽڱ仯����');
xlabel('ʱ��/s');
ylabel('�Ƕ�( ��)');
grid on;
hold on
plot(t_x,theta1_fig_1);
plot(t_x,theta1_fig_2);
plot(t_x,theta1_fig_3);
plot(t_x,theta1_fig_4);
legend('theta1','theta2','theta3','theta4');
hold off
%=======================================================================================================


theta_1 = 2*pi*theta1_fig_1/360;
theta_2 = 2*pi*theta1_fig_2/360;
theta_3 = 2*pi*theta1_fig_3/360;
theta_4 = 2*pi*theta1_fig_4/360;

clear theta1_fig_1 theta1_fig_2 theta1_fig_3 theta1_fig_4  zhi_1 zhi_2 zhi_3 zhi_4 val con

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
x = 1/sample2:time(SAMPLE)/sample2:time(SAMPLE);

subplot(3,1,2);
%figure(2)
title('���ؽڵ�����');
xlabel('ʱ��/s');
ylabel('����(��N��m��)');
grid on;
hold on
plot(x,t(1,:));
plot(x,t(2,:));
plot(x,t(3,:));
plot(x,t(4,:));
legend('�ؽ�1','�ؽ�2','�ؽ�3','�ؽ�4');
hold off

% 
clear P
for j=1:sample2-1
    P(1, j)=(t(1, j+1)-t(1,j))/STEP;
    P(2, j)=(t(2, j+1)-t(2,j))/STEP;
    P(3, j)=(t(3, j+1)-t(3,j))/STEP;
    P(4, j)=(t(4, j+1)-t(4,j))/STEP;
end


%����ƽ���Եı仯����
[temp1,temp2] = size(P);
%figure(3)
subplot(3,1,3);
%set(gca, 'Units', 'normalized', 'Position', [0.505 0.505 0.495 0.495])
title('���ؽ�ƽ���ԣ�б�ʣ�');
xlabel('ʱ��/s');
ylabel('N��m/S');
grid on;
hold on
plot(t_x(1:temp2),P(1,:));
plot(t_x(1:temp2),P(2,:));
plot(t_x(1:temp2),P(3,:));
plot(t_x(1:temp2),P(4,:));
legend('�ؽ�1','�ؽ�2','�ؽ�3','�ؽ�4');
hold off
P=abs(P);
%���ƽ����
% Pmax1_NO1=max(P(1,:))
% Pmax2_NO1=max(P(2,:))
% Pmax3_NO1=max(P(3,:))
% Pmax4_NO1=max(P(4,:))
%���ƽ�������ֵ







