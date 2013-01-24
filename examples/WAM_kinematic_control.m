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
thetastart =[0    0    0    0        0         0         0]';
%Final configuration
thetad = [1.7593    0.8796    0.1257   -1.4451   -1.0053    0.0628         0]';
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

