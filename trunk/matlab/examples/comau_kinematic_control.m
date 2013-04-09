clear all;
close all;
clear classes;
clc;

%% Definitions for DQ_kinematics
comau_DH_theta=  [0, -pi/2, pi/2, 0, 0, 0, pi];
comau_DH_d =     [-0.45, 0, 0, -0.64707, 0, -0.095, 0];
comau_DH_a =     [0, 0.150, 0.590, 0.13, 0, 0, 0];
comau_DH_alpha = [pi, pi/2, pi, -pi/2, -pi/2, pi/2, pi];
comau_dummy =    [0,0,0,0,0,0,1];

comau_DH_matrix = [comau_DH_theta;
    comau_DH_d;
    comau_DH_a;
    comau_DH_alpha;
    comau_dummy];

comau_kine = DQ_kinematics(comau_DH_matrix, 'modified');


%% Basic definitions for the simulation
initial_theta = [0,0,-pi/2,0,pi/2,0]';
desired_theta = [pi/2,0,-pi/2,0,pi/2,0]';

xd = comau_kine.fkm(desired_theta);

error = 1;
epsilon = 0.001;
K = 0.5;
theta = initial_theta;

figure;
hold on;
plot(comau_kine,theta);
plot(xd,'scale',0.5);


axis equal;
axis([-0.8,1.2,-0.8,0.8,-0.2,1.5]);
view(27,34);

pause(2);
while norm(error) > epsilon  
    xm = comau_kine.fkm(theta);
    J = comau_kine.jacobian(theta);
    error = vec8(xd-xm);
    theta = theta + pinv(J)*K*error;
   plot(comau_kine,theta);
end




