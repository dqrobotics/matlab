close all;
clear all;
clear classes;
clc;

%Standard D-H of KUKA-LWR
kuka_DH_theta=[0, 0, 0, 0, 0, 0, 0];
kuka_DH_d = [0.310, 0, 0.4, 0, 0.39, 0, 0];
kuka_DH_a = [0, 0, 0, 0, 0, 0, 0];
kuka_DH_alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];
kuka_DH_matrix = [kuka_DH_theta;
                  kuka_DH_d;
                  kuka_DH_a;
                  kuka_DH_alpha];
                              
kuka = DQ_kinematics(kuka_DH_matrix,'standard');

%Initial configuration
thetastart =[0    0.3770    0.1257   -0.5655         0         0         0]';
%Final configuration
thetad = [1.7593    0.8796    0.1257   -1.4451   -1.0053    0.0628         0]';
theta = thetastart;

epsilon = 0.001; %The error must be bellow this value in order to stop the robot
gain = 0.1; %Gain of the controllers

xd = kuka.fkm(thetad); %Desired end-effector's pose

figure;
axis equal;
plot(kuka, theta);

grid off;
view(-0,0)
hold on;

plot(kuka, theta);
    

fprintf('Performing standard kinematic control using dual quaternion coordinates');
xm = kuka.fkm(theta);
error = epsilon+1;
while norm(error) > epsilon
    jacob = kuka.jacobian(theta);
    xm = kuka.fkm(theta);
    error = vec8(xd-xm);
    theta = theta+pinv(jacob)*gain*error;
    plot(kuka, theta');    
end

fprintf('\nNow let us control only the translation part\n');
%The end-effector will touch the base
pd = [0,0,0,0];

error = epsilon+1;
while norm(error) > epsilon
    jacob = kuka.jacobian(theta);
    xm = kuka.fkm(theta);
    jacobp = kuka.jacobp(jacob,xm);
    pm = translation(xm);
    error = vec4(pd-pm);    
    theta = theta+pinv(jacobp)*gain*error;
    plot(kuka, theta');       
end

fprintf('\nNow let us control only the orientation\n')

%The end-effector will be aligned with the world frame
rd = DQ(1);

error = epsilon+1;
while norm(error) > epsilon
    jacob = kuka.jacobian(theta);
    xm = kuka.fkm(theta);
    jacobr = jacob(1:4,:);
    rm = xm.P;
    error = vec4(rd-rm);    
    theta = theta+pinv(jacobr)*gain*error;
    plot(kuka, theta');       
end


fprintf('\nNow let us place the end-effector at a distance of 0.2 m from the base (we are going to perform distance control)\n')

%Technically speaking, we're controlling the square of the distance,
%otherwise the distance Jacobian can have singularities. See discussion on page 76 of 
%ADORNO, B. V., Two-arm manipulation: from manipulators to enhanced human-robot
% collaboration, Université Montpellier 2, Montpellier, France, 2011.
dd=0.2^2;
error = epsilon+1;
while norm(error) > epsilon
    jacob = kuka.jacobian(theta);
    xm = kuka.fkm(theta);
    jacobd = kuka.jacobd(jacob,xm);
    dm = norm(vec4(translation(xm)))^2;
    error = dd-dm;    
    theta = theta+pinv(jacobd)*gain*error;
    plot(kuka, theta');       
end





