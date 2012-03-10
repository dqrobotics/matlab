clear all;
close all;
clear classes;
clc;


%L(i)= LINK([alpha     A   theta        D     sigma offset], CONVENTION)

L{1} = link([ pi     0       0       -0.450     0   0], 'modified');
L{2} = link([ pi/2   0.150   0       0          0   -pi/2], 'modified');
L{3} = link([ pi     0.590   0       0          0   pi/2], 'modified');
L{4} = link([ -pi/2  0.130   0       -0.64707	0	0], 'modified');
L{5} = link([-pi/2   0       0       0          0   0], 'modified');
L{6} = link([ pi/2 	 0       0       -0.095     0   0], 'modified');
L{7} = link([ pi 	 0       0       0          0   pi], 'modified');

cs6 = robot(L, 'Smart Six', 'Comau');

cs6.name = 'Smart Six';
cs6.manuf = 'Comau';


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
epsilon = 0.01;
K = 0.1;
theta = initial_theta;

figure;
hold on;
plot(cs6,[theta;0]');
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
    plot(cs6,[theta;0]');
end




