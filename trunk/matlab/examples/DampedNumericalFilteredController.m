clear all;
close all;
clc;
format SHORTE;

%% Chiaverini's Controller Implementation
% Uses variable damping (lambda) with numerical filtering along with a
% isotropic damping (beta).

%% Constant Initialization.
% Controller Step Maximum
control_step_max = 200;
% Constants
pi2 = pi/2;
% Controller Gains And Dampings
lambda_max  = 0.01;  %Variable Damping
beta        = 0.01;  %Isotropic Damping
epsilon     = 0.001; %Singular region size
kp = diag([0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8]); %Proportional Gain
% Robot Initial Positions
thetas = [0;pi2;0;0;0;0;0];
% Desired End Effector Final Pose
xd = DQ([1,0,0,0,0,0,0,0.652495]);

%% Robot Declaration
schunk_dh = [0,     0,   0,     0,   0,      0,  0;
	         0.3,   0,   0.328, 0,   0.2765, 0,  0.40049;
	         0,     0,   0,     0,   0,      0,  0;
	        -pi2,   pi2,-pi2,   pi2,-pi2,    pi2,0];
schunk = DQ_kinematics(schunk_dh, 'standard');
dofs = schunk.links;

% Auxiliar variables
identity = eye(8,8);

%% Loops for a given amount of steps
for i=1:control_step_max
   
    x = schunk.fkm(thetas);
    J = schunk.jacobian(thetas);
    
    [U,S,V] = svd(J);
    sigma_min = S(dofs,dofs);
    u_m = U(:,dofs);
        
    lambda = lambda_max;
    if sigma_min < epsilon
        lambda = (1-(sigma_min/epsilon)^2)*(lambda_max^2);
    end
    
    % Base dislocation invariant error
    error = vec8(1 - x'*xd);
    J_inv = (J')/(J*(J') + (beta*beta)*identity + (lambda*lambda)*u_m*(u_m'));
    
    delta_thetas = J_inv*kp*error;
    thetas = thetas + delta_thetas;

end

