% Simple example on how to define a serially coupled heterogeneous
% kinematic chain.

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

function robot_serialization(varargin)
    %default behavior is to visualize the trajectory
    visualize = 1; 
    if nargin == 1 && strcmp(varargin{1}, 'novisual')
        visualize = 0;
    end

    base = DQ_HolonomicBase;
    arm = {KukaLwr4Robot.kinematics(),...
           KukaLwr4Robot.kinematics(),...
           Ax18ManipulatorRobot.kinematics(),...
           KukaLwr4Robot.kinematics(),...
           Ax18ManipulatorRobot.kinematics()};

    %% Initializes the robot with base
    robot = DQ_WholeBody(base);
    % Serially add all the arms
    for i = 1:length(arm)
        robot.add(arm{i});
    end

    %% Initialize the configuration vector. All variables are zero except three.
    % This is completely arbitrary.
    q = zeros(robot.get_dim_configuration_space(),1);
    q(1) = 3;
    q(5) = pi/2; % second joint of the first arm
    q(12) = -pi/2; % second joint of the second arm

    %% Initalize the plot
    % Plot the global reference frame
    h = figure;
    plot(DQ(1));
    hold on;
    % Plot the robot in the initial configuraion
    plot(robot,q);
    axis square;
    axis([-10,10,-10,10,0,5]);
    hold on;
    title('Starting the robot motion in 3 seconds.')
    pause(3);
    title('Press ''q'' to quit');

    %% Initializing the keyboard 'stroke capturing system' with a value
    % different from  'q'
    key = '@';
    set(gcf,'CurrentCharacter',key);

    %% The robot will move until the user presses 'q'
    i = 0;
    
    while key ~= 'q'     
        key = get(gcf,'CurrentCharacter');
        i = mod(i + 0.1, 2*pi); % After 2*pi, i goes again to zero
        q(1) = 3*cos(i); % x-coordinate of the mobile base
        q(2) = 3*sin(i); % y-coordinate of the mobile base
        q(3) = i; % phi angle of the mobile base
        q(4) = i; % first joint of the first arm

        plot(robot,q); 
        drawnow;
        if (~visualize) & (i > pi)
            key = 'q';
        end     
    end
    
    close(h);
end
