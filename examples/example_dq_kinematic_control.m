% (C) Copyright 2017 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br


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
% This is the variable used in the parsimonious controller to enforce null 
% joints velocities when the error goes to zero
controller_variables.beta = 1; 

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
controller_type = 'pseudoinverse';
while ~strcmp(controller_type,'none')
    
    %For all controllers, the robot starts at the same configuration
    controller_variables.q = [0; 0.3770; 0.1257; -0.5655; 0; 0; 0];
    ii = 1;
    e = epsilon + 1;
    
    switch controller_type
        case 'pseudoinverse'
            tic 
            while norm(e) > epsilon 
                [u,e] = kinematic_controller.pseudoinverse_pose_controller(controller_variables);                 
                controller_variables.q = controller_variables.q + u; %integrates the control input and update the structure controller_variables
                % Visualization
                plot(kuka, controller_variables.q);  
                drawnow;
                ii = ii + 1;
            end
            toc
            fprintf('\n Number of iterations for the pseudoinverse controller: %f\n', ii);
            controller_type = 'damped_pseudoinverse';
            
        case 'damped_pseudoinverse'
            tic 
            while norm(e) > epsilon 
                [u,e] = kinematic_controller.damped_pseudoinverse_pose_controller(controller_variables);                 
                controller_variables.q = controller_variables.q + u; %integrates the control input and update the structure controller_variables
                % Visualization
                plot(kuka, controller_variables.q);  
                drawnow;
                ii = ii + 1;
            end
            toc
            fprintf('\n Number of iterations for the damped pseudoinverse controller: %f\n', ii);
            controller_type = 'parsimonious';
            
        case 'parsimonious'
            tic 
            while norm(e) > epsilon 
                [u,e] = kinematic_controller.parsimonious_pose_controller(controller_variables);                 
                controller_variables.q = controller_variables.q + u; %integrates the control input and update the structure controller_variables
                % Visualization
                plot(kuka, controller_variables.q);  
                drawnow;
                ii = ii + 1;
            end
            toc
            fprintf('\n Number of iterations for the parsimonious controller: %f\n', ii);
            controller_type = 'none';
    end
end






