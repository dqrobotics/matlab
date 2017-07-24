close all;
clear all;
clear classes;
clc;

% Create a new DQ_kinematics object with KUKA LWR parameters
kuka = DQ_KUKA;

% Initial configuration
q = [0; 0.3770; 0.1257; -0.5655; 0; 0; 0];
% Since we don't care about a specific desired pose, we just pick an
% arbitrary configuration to generate the desired pose
qd = [1.7593; 0.8796; 0.1257; -1.4451; -1.0053; 0.06280; 0];
xd = kuka.fkm(qd); % Desired end-effector's pose


% Setting up the controller
kinematic_controller = DQ_KinematicController(kuka);
controller_variables.K = 0.1; % Controller gain
controller_variables.q = q; % Current robot configuration 
controller_variables.xd = xd; % Desired end-effector pose
controller_variables.lambda = 0.09; % This is the damping factor for the damped_pseudo_inverse controller;
% Stop criterion. The error must be bellow this value in order to stop the 
% robot.
epsilon = 0.001; 

% Setting up the visualization
figure;
axis equal;
plot(kuka, q, 'nobase');
view(-0,0);
grid off;

% Let's control our robot
e = epsilon + 1;
while norm(e) > epsilon    
    [u,e] = kinematic_controller.damped_pseudoinverse_pose_controller(controller_variables); 
    controller_variables.q = controller_variables.q + u; %integrates the control input and update the structure controller_variables
    
    % Visualization
    plot(kuka, controller_variables.q);  
    drawnow;
end






