%========================================================
% robotic toolbox的操作的笔记：
% copyright：wu bing
% author:wu bing
% time-01：2020-05-18 
% time-02：2020-05-19
%=========================================================
%note_1:如果matlab识别不了命令，就去matlab安装路径下找到toolbox文件夹再进入robotic-toolbox的文件夹里面有个startup_rvc.m 文件直接拖到matlab命令行自动安装即可。
%note_2:同样的文件夹下，有robot子文件夹，里面有robot.pdf文件，打开，不懂得命令直接里面查找说明就行。

clc
clear
close all

%——————————————————————————————————————————
% 1.二维空间位姿描述命令
x = 2;y=2;theta=pi/4;
T = SE2(x,y,theta);                  %代表（x，y）的平移和theta角度的旋转
trplot2(T);                           %画出相对世界坐标系的变换T
T = transl2(x,y);                     %二维空间中，纯平移的其次变换
%——————————————————————————————————————————
% 2.三维空间位姿描述命令
x = 2;y=2;z=2;theta=pi/4;
T_3x3_1 = rotx(theta);
T_3x3_2 = rotx(theta+pi/6);
T_3x3_3 = rotx(theta+pi/7);           %绕xyz轴旋转theta得到旋转矩阵（3x3）
T_3x3 = T_3x3_1*T_3x3_2*T_3x3_3;      %相互相乘就可以叠加效果 tips：注意坐标系左乘和右乘的不同！
trplot(T_3x3_1);                      %绘制出相应的旋转矩阵
tranimate(T_3x3);                     %实现一个旋转的动画效果
T_3x3 = transl([x,y,z]);              %实现坐标的平移变换
T_4x4 = trotx(theta);
T_4x4 = troty(theta);
T_4x4 = trotz(theta);                 %实现绕xyz轴旋转theta的齐次变换矩阵（4x4）
tranimate(T_4x4);
%——————————————————————————————————————————
% 3.建立机器人模型
% 3.1 link类根据机器人的关节生成对应的机器臂
%     R= Link（[theta，d，a，alpha]） D-H法建立机器人对应的参数 关节角、连杆偏距、连杆长度、连杆转角
%         D-H参数法建的坐标系z轴为转轴，x轴有此回转圆心指向下一回转圆心
%         关节角（转角）[theta]：x_i-1和x_i之间的夹角
%         连杆偏距[d]：沿z轴的长度
%         连杆长度[a]：沿x轴的长度
%         连杆转角[alpha]：z_i-1和z_i之间的夹角 
% ------------------------------------------------------
% |使用Link命令后会生成link类的R，具有以下的属性           |
% |R.RP:获取连杆关节的类型      R.theta：获取连杆的关节角  |
% |R.d：获取连杆偏距            R.a：获取连杆长度          |
% |R.alpha：获取连杆扭转角                                |
%---------------------------------------------------------
theta = [pi/4 pi/3 pi/6 ];
L(1) = Link([pi/2 6 0 pi/2]);
L(2) = Link([0 0 6 0]);
L(3) = Link([0 0 6 0]);
L(3).RP;                                   % L(3).type()函数代替
L.theta; 
%——————————————————————————————————————————
% 3.2 SerialLink类
% L = SerialLink(links,options)将连杆组成机械臂
% SerialLink有只读属性和读写属性两个属性，具体可以自己翻手册，懒得写了，可以更改机器人的名字，重力方向什么的。
triDOF_robot = SerialLink(L);
triDOF_robot.teach;                                   %示教界面
triDOF_robot.display;                                 %显示D-H的四个参数
triDOF_robot.plot(theta);                             %画出为theta角度的机器人图
%——————————————————————————————————————————
%3.3 运动学和Jacob矩阵
%这些命令都是针对机械臂而言的.
%运动学
% fkine([theta]) 根据各个关节的theta求解出末端姿态【正运动学】
% ikine6s() 给定末端的姿态求解关节角度【逆运动学】【逆运动学封闭解（不知道啥意思解析解??）】[6轴]
% ikine() 给定末端的姿态求解关节角度【逆运动学】【逆运动学数值解】
%######################################################################
clc
clear
close all

%建立机械臂,并限制角度
L(1) = Link([pi/2 6 0 pi/2]);
L(2) = Link([0 0 6 0]);
L(3) = Link([0 0 6 0]);
% L(1).qlim = [deg2rad(-90) deg2rad(90)];
% L(2).qlim = [deg2rad(0) deg2rad(85)];
% L(3).qlim = [deg2rad(-90) deg2rad(10)];
triDOF_robot = SerialLink(L);

%轨迹图象的建立
%1.直线
T1 = transl(12,0,6); %起点
T2 = transl(2.458,7.565,13.417);%终点
T = ctraj(T1,T2,50);  %ctraj(T1,T2,n)两个姿态间的笛卡尔坐标系下的轨迹 4x4xn 的变换矩阵形式给出
T3 = transl(9.628,-4.826,4.486);
T4 = ctraj(T2,T3,50);
% tranimate(T);
%逆运动计算
inv_theta_1 = triDOF_robot.ikine(T,'mask',[1 1 1 0 0 0]);
inv_theta_2 = triDOF_robot.ikine(T4,'mask',[1 1 1 0 0 0]);
triDOF_robot.plot(inv_theta_1);
triDOF_robot.plot(inv_theta_2);     %可以多端直线组合。
%——————————————————————————
%                 attiention
% ikine()是只支持六轴的逆运动，对于六轴以下的运动需要加mask vector来忽略一些自由度。
% [x平移 y平移 z平移 x旋转 y旋转 z旋转]
%——————————————————————————

%#####################################################################
theta = [pi/4 pi/3 pi/6 ];
T_0_n = triDOF_robot.fkine(theta);          %关节角度为theta是，末端坐标系相对于基坐标系的变换矩阵（4x4）
tranimate(T_0_n);


%雅克比矩阵
% jacob0()在基座标系下的jacob矩阵        
% jacobn()在末端坐标系下的jacob矩阵


































