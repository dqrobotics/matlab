clear all;
close all;
clear classes;
clc;

%Create a new DQ_kinematics object with the Denso standard Denavit-Hartenberg parameters           
denso_kine = DQ_DENSO;

% Basic definitions for the simulation
theta = [0,0,0,0,0,0]';      %initial configuration
% Move Arm

position = [0.4,0.0,0.02]

thetad = [0,0,0,0,0,0]';

phi = pi/2;

n_y =1;
n_x =0;
n_z =0;

n_vec = [n_x,n_y,n_z];
n = n_vec/norm(n_vec);

n = DQ([0, n_vec(1), n_vec(2), n_vec(3)]);

theta1 = atan2(position(2), position(1));
rz = cos(theta1/2) + sin(theta1/2)*DQ([0, 0, 0, 1]);

ry = cos(pi/4) + sin(pi/4)*DQ([0, 0, 1, 0]);

r = cos(phi/2) + sin(phi/2)*n;

r = ry * r;

p = DQ([0, position(1), position(2), position(3)]);

xd = r + 1/2 * DQ.E * p * r;

epsilon = 0.001;
K = 0.5;
error = epsilon+1;
lambda = 0.5;

pause(1);

while norm(error) > epsilon  
    jacob = denso_kine.jacobian(theta);
    xm = denso_kine.fkm(theta);
    error = vec8(xd-xm);
    
    jacob_pinv = (transpose(jacob)/(jacob*transpose(jacob) + (lambda^2)*eye(8)));
        
     
    theta = theta + K*jacob_pinv*error;
    
    norm(error)
    plot(denso_kine, theta');
    plot(xm,'scale',0.2);
    hold on
    axis equal;
    axis([-0.8,1.2,-0.8,0.8,-0.2,1.5]);
    view(-0.5 ,0);
    drawnow;

end

