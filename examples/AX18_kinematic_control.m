close all;
clear all;
clear classes;
clc;

%Create a new DQ_kinematics object with the AX18 arm standard Denavit-Hartenberg parameters           
ax18 = DQ_AX18;

%Initial configuration
theta=[0 0 0 0 0]'; 

%Configuring the end-effector:
ax18.set_effector(cos(pi/4)+DQ.i*sin(pi/4))

p = translation(ax18.fkm(theta));
pd = translation(ax18.fkm([pi/4 -pi/4 pi/2 0 pi/4]));

error = pd - p;
while norm(vec4(error)) > 0.1
    x = ax18.fkm(theta);
    p = translation(x);
    Jp = ax18.jacobp(ax18.jacobian(theta),x);
    error = pd - p;
    theta = theta + pinv(Jp)*0.1*vec4(error);
    plot(ax18,theta);
    drawnow;
end
