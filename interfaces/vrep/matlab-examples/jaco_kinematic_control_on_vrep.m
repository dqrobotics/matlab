% Performs the kinematic control of the JACO robot using V-REP.
%
% Usage:
%       1) Open scene "jaco_position_actuation.ttt" on V-REP.
%       2) Run this file.

% (C) Copyright 2020 DQ Robotics Developers
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
%     Frederico Fernandes Afonso Silva - fredf.afonso@gmail.com

close all
clear class
clear all %#ok
clc


%% V-REP interface
vi = DQ_VrepInterface();
vi.connect('127.0.0.1',19997);
disp('Communication established!')
vi.set_synchronous(true);
vi.start_simulation();
disp('Simulation started!')
pause(1)

%% Robot Definition (DQ Robotics)
robot = JacoVrepRobot("Jaco", vi);
n = size(robot.kinematics().theta,2);

% Initial joint positions
q = robot.get_q_from_vrep;

% Desired end-effector pose
qd = [0.0; 3.5; 2.0; 4.2; 1.4; 0.0];
xd = robot.kinematics().fkm(qd);


%% General variables
%  Stop criteria
stability_counter = 0;
stability_counter_max = 2000;
error_dot = 10;
stability_threshold = 1e-4;%1e-5;

% Controller variables
k = 1;
T = 0.05;
x = robot.kinematics().fkm(q);
error = vec8(xd-x);

%% Main Loop
joint_names = {'Jaco_joint1','Jaco_joint2','Jaco_joint3','Jaco_joint4','Jaco_joint5','Jaco_joint6'};
iteration = 1;
while(stability_counter < stability_counter_max)
    % Control
    J = robot.kinematics().pose_jacobian(q);
    x = robot.kinematics().fkm(q);
    
    error_old = error;
    error = vec8(xd-x);
    q_dot = pinv(J)*k*error;
    
    % V-REP communication
    q_read =  robot.get_q_from_vrep;
    q = q_dot*T + q_read;
    robot.send_q_to_vrep(q');
    vi.trigger_next_simulation_step(); % force sensor returns noise values if called before the first trigger
    vi.wait_for_simulation_step_to_end();
    
    % Read joint velocities
    qd_read = robot.get_q_dot_from_vrep();
    formatSpec = 'Joint velocities read from V-REP: %f\n';
    fprintf(formatSpec,qd_read);

    % Storage of torque
    storage_norm_error(:,iteration) = norm(error); %#ok
        
    formatSpec = 'Norm of the error: %f\n';
    fprintf(formatSpec,norm(error));
    
    formatSpec = 'Norm of the error_dot: %f\n';
    fprintf(formatSpec,norm(error_dot));

    iteration = iteration + 1;
    formatSpec = 'Step: %f\n';
    fprintf(formatSpec,iteration);
    
    % Check if simulation should stop
    error_dot = error_old - error;
    if(double(norm(error_dot)) < stability_threshold)
        stability_counter = stability_counter + 1;
    else
        stability_counter = 0;
    end
    formatSpec = 'Converged for %i steps.\n';
    fprintf(formatSpec,stability_counter);
end

%% Display graph of the norm of the error
plot(storage_norm_error,'b');
xlabel('Number of iterations')
ylabel('Norm of the error')

% Finish V-REP communication
vi.stop_simulation();
disp('Simulation finished!')
vi.disconnect_all();
disp('Communication finished!')