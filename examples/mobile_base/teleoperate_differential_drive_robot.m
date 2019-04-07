% Simple example to test the DQ_DifferentialDriveRobot class.
%
% teleoperate_differential_drive_robot() starts the teleoperation of the simulated
% robot with default parameters; that is, wheel radius equal to 0.1 m and
% distance between wheels equal to 0.4 m.
%
% teleoperate_differential_drive_robot(wheel_radius, distance_between_wheels)
% start the teleoperation of the simulated robot with parameters given by 
% wheel_radius and distance_between_wheels, both in meters.

% (C) Copyright 2019 DQ Robotics Developers
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
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br

function teleoperate_differential_drive_robot(varargin)

    %% Configuring the nonholonomic mobile base
    if nargin == 0
        wheel_radius = 0.1; % Wheel radius (meters)
        distance_between_wheels = 0.4; % Distance between wheels (meters)
    elseif nargin == 2
        wheel_radius = varargin{1};
        distance_between_wheels = varargin{2};
    else
        error_msg = sprintf(...
        ['\nUsage: \n'...        
        'teleoperate_differential_drive_robot() starts the teleoperation of'... 
        ' the simulated robot with default parameters; that is, wheel radius'...
        ' equal to 0.1 m and distance between wheels equal to 0.4 m.\n\n'...
        'teleoperate_differential_drive_robot(wheel_radius,'...
        ' distance_between_wheels) start the teleoperation of the simulated'...
        ' robot with parameters given by wheel_radius and '...
        'distance_between_wheels, both in meters.']); 
        error(error_msg);
    end

    base = DQ_DifferentialDriveRobot(wheel_radius, distance_between_wheels);
    
    fprintf(['\nStarting simulation of a differential drive robot with wheel'...
        'radius equal to %f m and distance between wheels equal to %f m'], ...
        wheel_radius, distance_between_wheels);

    q = [1,1,0]'; % Initial configuration
    T = 0.001; % Integration step for the animation

    %% Scene setup
    %Key-press events are handled by the function keypress.
    fig_handle = figure('KeyPressFcn',@keypress);
    hold on;
    axis equal;
    axis(distance_between_wheels*[-10,10,-10,10,0,1/distance_between_wheels]);    
    view(3);
    xlabel('X');
    ylabel('Y');
    title(['Press ''q'' to quit. Use the keyboard arrows to teleoperate the '...
        'robot.']);


    % The struct S is shared between the main function and the keypress
    % function.
    S.u = [0;0]; % Velocity inputs. The robot is intially stopped
    S.vel = 100; % Wheels angular velocity step
    S.quit = '@'; % The program quits when this variable equals '@'

    %Store the struct in the figure
    guidata(fig_handle,S);

    %% Robot teleoperation with the keyboard
    while S.quit ~= 'q'
        % Get the information updated by the keypress() function
        S = guidata(fig_handle);
        % update robot configuration and draw it
        q = q + T*base.constraint_jacobian(q(3))*S.u;
        plot(base,q);
        drawnow;
    end

    %% Time to quit
    close(fig_handle);
    clc;   
    disp('That''s it, I''m out!');
    clear fig_handle;
    return;
end

% keypress() handles the keyboard events
function keypress(H,E)
    S = guidata(H);
    
    %Self-explanatory state machine.
    switch E.Key
        case 'rightarrow'
            S.u = S.u + [-S.vel;S.vel];

        case 'leftarrow'
            S.u = S.u - [-S.vel;S.vel];

        case 'uparrow'
            S.u = S.u + S.vel;

        case 'downarrow'
            S.u = S.u - S.vel;
            
        case 'q'
            S.quit = 'q';
    end
    
    % Store the updated control inputs and the program status (S.quit)
    guidata(H,S);
end
