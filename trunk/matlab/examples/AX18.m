
clear all;
clear classes;
clc;

figure

% D&H Parameters Vectors
DH_Alpha   = [-pi/2  0       -pi/2  0      -pi/2   0];  % Alpha
DH_A       = [0      159     0      22.25   0      0];   % A
DH_Theta   = [0      0      -pi/2   0      -pi/2   0];  % Theta
DH_D       = [167    0       0      81.5    41     0];    % D
DH_virtual = [0,     0,      0,     1,      0,     0];         % Virtual joint definition

i=6;
DH_RobotMatrix = [ DH_Theta(1:i) ; DH_D(1:i) ; DH_A(1:i) ; DH_Alpha(1:i) ; DH_virtual(1:i) ]; % D&H parameters matrix for the arm model

ax18 = DQ_kinematics(DH_RobotMatrix,'standard'); % Defines robot model using dual quaternions

theta0=[0 0 0 0 0]; %Initial Position for the Robot Arm

plot(ax18,theta0);