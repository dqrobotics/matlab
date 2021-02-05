% Performs the joint torque actuation of the JACO robot using V-REP.
%
% Usage:
%       1) Open scene "jaco_torque_actuation.ttt" on V-REP.
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

%% Robot Definition (DQ Robotics)
robot = JacoVrepRobot("Jaco", vi);
% robot = LBR4pVrepRobot("LBR4p", vi);

%% Main Loop
iteration = 1;
last_iteration = 20;
while(iteration < last_iteration)
    % V-REP communication
    tau = -2*pi*rand(6,1); % random joint torques
    robot.send_tau_to_vrep(tau);
    vi.trigger_next_simulation_step();
    vi.wait_for_simulation_step_to_end();
    
    formatSpec = 'Joint torque sent: %f\n';
    fprintf(formatSpec,tau');
    
    tau_read = robot.get_tau_from_vrep();
    formatSpec = 'Joint torque read from V-REP: %f\n';
    fprintf(formatSpec,tau_read');

    iteration = iteration + 1;
    formatSpec = 'Step: %f\n';
    fprintf(formatSpec,iteration);
end

% Finish V-REP communication
vi.stop_simulation();
disp('Simulation finished!')
vi.disconnect_all();
disp('Communication finished!')