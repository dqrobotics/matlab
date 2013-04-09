close all;
clear all;
clear classes;
clc;

%Standard D-H of WAM
wam_DH_theta=[0, 0, 0, 0, 0, 0, 0];
wam_DH_d = [0, 0, 0.55, 0, 0.3, 0, 0.0609];
wam_DH_a = [0, 0, 0.045, -0.045, 0, 0, 0];
wam_DH_alpha =  pi*[-0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0];
wam_DH_matrix = [wam_DH_theta;
                  wam_DH_d;
                  wam_DH_a;
                  wam_DH_alpha];
                              
wam = DQ_kinematics(wam_DH_matrix,'standard');

%Initial configuration
thetastart =[-2.65    -0.604    -0.46    2.68        1.35         1.61         -3.03]';
%Final configuration
thetad = [-2.0    -0.304    -0.3    2.3        1.0         1.0         -2.03]';
theta = thetastart;

epsilon = 0.001; %The error must be bellow this value in order to stop the robot
gain = 0.1; %Gain of the controllers

xd = wam.fkm(thetad); %Desired end-effector's pose

figure;
axis equal;
plot(wam, theta);

grid off;
view(-0,0)
hold on;

plot(wam, theta);
pause

fprintf('Performing standard kinematic control using dual quaternion coordinates');
xm = wam.fkm(theta);
error = epsilon+1;
while norm(error) > epsilon
    jacob = wam.jacobian(theta);
    xm = wam.fkm(theta);
    error = vec8(xd-xm);
    theta = theta+pinv(jacob)*gain*error;
    plot(wam, theta');    
end

