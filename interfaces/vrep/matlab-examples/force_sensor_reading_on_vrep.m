% Performs the readings of a force sensor and calls a script function in V-REP.
%
% Usage:
%       1) Open scene "simple_weighing_scale.ttt" on V-REP.
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
vi.trigger_next_simulation_step(); % force sensor returns noise values if called before the first trigger
vi.wait_for_simulation_step_to_end();

% Set force sensor name
force_sensor_name = 'Force_sensor';

%% Get mass of the scene objects
% Get the mass of the weighing plate
handle_plate = vi.get_handles({'Plate'});
[return_code,~,mass_as_float,~,~] = vi.call_script_function('remote_api_command_server',vi.ST_CHILD,'get_mass',handle_plate,[],[],[]);
if(return_code == vi.CRC_OK)
    mass_plate = mass_as_float;
end

% Get the mass of the cube
handle_cube = vi.get_handles({'Cube'});
[return_code,~,mass_as_float,~,~] = vi.call_script_function('remote_api_command_server',vi.ST_CHILD,'get_mass',handle_cube,[],[],[]);
if(return_code == vi.CRC_OK)
    mass_cube = mass_as_float;
end

% Gravity acceleration
g = -9.81;

%% Main loop
iteration = 0;
final_iteration = 20;
while(iteration < final_iteration)
    % V-REP communication
    vi.trigger_next_simulation_step();
    vi.wait_for_simulation_step_to_end();
    
    %% Get force sensor readings
    [force_vec, torque_vec] = vi.get_force_sensor_readings(force_sensor_name);
    wrench_read = DQ([0 force_vec 0 torque_vec]);
    formatSpec = 'Wrench read from forcen sensor on V-REP: %f\n';
    fprintf(formatSpec,wrench_read.vec6');
    
    formatSpec = 'You are supposed to read as force on z-axix (third element from the wrench read): %f\n';
    fprintf(formatSpec,(g*(mass_plate + mass_cube)));
    disp('Other values are sensor noise and/or caused by the cube shaking during simulation.');
    disp('Those values should be close to zero.');
    disp(' ');
    
    iteration = iteration + 1;
    formatSpec = 'Step: %f\n';
    fprintf(formatSpec,iteration);
end

%% Finishes V-REP communication
vi.stop_simulation();
disp('Simulation finished!')
vi.disconnect_all();
disp('Communication finished!')