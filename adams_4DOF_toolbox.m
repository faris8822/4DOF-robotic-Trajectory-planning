clc
clear
close all
% 
% GANG_1 = 1200;
% GANG_2 = 1300;
% GANG_3 = 1300;
% GANG_4 = 1300;

% GANG_1 = 900;
% GANG_2 = 1300;
% GANG_3 = 1300;
% GANG_4 = 1700;

% GANG_1 = 900;
% GANG_2 = 1600;
% GANG_3 = 1600;
% GANG_4 = 1100;

GANG_1 = 700;
GANG_2 = 1600;
GANG_3 = 1600;
GANG_4 = 1300;


L(1) = Link([0 0 GANG_1 0]);
L(2) = Link([0 0 GANG_2 0]);
L(3) = Link([0 0 GANG_3 0]);
L(4) = Link([0 0 GANG_4 0]);

robot_4DOF = SerialLink(L);

theta = [pi/2 -pi/2 pi/2 -pi/2];
% theta= [0 0 0 0];
J = robot_4DOF.jacob0(theta);
% robot_4DOF.jacobe(theta1)
F = [2500 0 0 0 0 0];
t = J'*F'
 robot_4DOF.plot(theta)

