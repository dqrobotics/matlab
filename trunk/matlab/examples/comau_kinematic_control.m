clear all;
close all;
clear classes;
clc;

%Create a new DQ_kinematics object with the COMAU SmartSiX modified Denavit-Hartenberg parameters           
comau_kine = DQ_COMAU;


%% Basic definitions for the simulation
theta = [0,0,-pi/2,0,pi/2,0]';      %initial configuration
thetad = [pi/2,0,-pi/2,0,pi/2,0]';  %desired configuration

%desired end-effector pose
xd = comau_kine.fkm(thetad);
%Plot the desired pose
plot(xd,'scale',0.5);

error = 1;
epsilon = 0.001;
K = 0.5;


%Plot the robot in the initial configuration
hold on;
plot(comau_kine,theta);


%Configure the axis for a good visualisation
axis equal;
axis([-0.8,1.2,-0.8,0.8,-0.2,1.5]);
view(27,34);

pause(1);
while norm(error) > epsilon  
    x = comau_kine.fkm(theta);
    J = comau_kine.jacobian(theta);
    error = vec8(xd-x);
    theta = theta + pinv(J)*K*error;
    plot(comau_kine,theta);
    drawnow;
end




