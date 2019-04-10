% Simple example on how to perform whole body control. We use a holonomic
% mobile base serially coupled to five manipulators, all in series. Since
% the overall robot has 34 DOF, first we simulate and then we show the
% robot motion to improve the visualization. The desired end-effector pose
% is randomly generated.

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

function whole_body_control_example(varargin)
    %default behavior is to visualize the trajectory
    visualize = 1; 
    if nargin == 1 && strcmp(varargin{1}, 'novisual')
        visualize = 0;
    end
    
    % Include the DQ name space to use i_ instead of DQ.i, etc.
    include_namespace_dq

    % The components of our mobile manipulator
    base = DQ_HolonomicBase;
    arm = {KukaLwr4Robot.kinematics(),...
           KukaLwr4Robot.kinematics(),...
           Ax18ManipulatorRobot.kinematics(),...
           KukaLwr4Robot.kinematics(),...
           Ax18ManipulatorRobot.kinematics()};

    %% Initializes the robot serially coupled kinematic chain from the mobile base
    robot = DQ_WholeBody(base);
    % Serially add all the arms
    for i = 1:length(arm)
        robot.add(arm{i});
    end

    %% Initialize the configuration vector. 
    % All variables are zero except four. This is completely arbitrary.
    q = zeros(robot.get_dim_configuration_space(),1);
    q(1) = 3;
    q(5) = pi/2; % second joint of the first arm
    q(6) = -pi/2;
    q(12) = -pi/2; % second joint of the second arm

    %% Initialize controller parameters and control objective
    x = robot.fkm(q); % Initial end-effector pose

    % Define a rotation of pi/2 around a random rotation axis
    r = cos(pi/2) + normalize(DQ(rand(3,1)))*sin(pi/2);
    % Define a random translation
    p = 15*rand(1)*i_ + (15*rand(1)-7.5)*j_ + (0.5*rand(1) + 1)*k_;

    % In order to define a decoupled transformation we use a CMI(3)
    % multiplication, also known as decompositional multiplication
    xd = (r + E_*(1/2)*p*r) .* x;


    %% Initalize the plot
    % Plot the global reference frame
    plot(DQ(1));
    hold on;
    plot(xd,'scale',0.2);

    % Plot the robot in the initial configuraion
    plot(robot,q);
    %axis square;
    axis([-2,15,-15,15,0,3.1]);
    hold on;
    title('Start the simulation in 3s. Press ''q'' anytime to abort and quit');
    pause(1);
    title('Start the simulation in 2s. Press ''q'' anytime to abort and quit');
    pause(1);
    title('Start the simulation in 1s. Press ''q'' anytime to abort and quit');
    pause(1);

    %% Initializing the keyboard 'stroke capturing system' with a value
    % different from  'q'
    key = '@';
    set(gcf,'CurrentCharacter',key);

    %% The simulation will run until the user presses 'q'
    x_error = 1;
    T = 0.001; % Integration step used in the robot configuration update
    gain = 100; % The gain determines the convergence rate
    i = 1;
    while key ~= 'q' && norm(x_error) > 0.01

        my_text = sprintf(['After the error becomes less than 0.01,'...
            ' the visualization will begin. Current error is %f'],norm(x_error));
        title(my_text)
        key = get(gcf,'CurrentCharacter');

        %% This is the part related to the robot motion control    
        x = robot.fkm(q);
        J = robot.pose_jacobian(q);
        N = haminus8(xd)*DQ.C8*J;
        x_error = vec8(x'*xd - 1);    
        u = -pinv(N)*gain*x_error;
        % Which ends here!

        % Store the current configuration to visualize it later
        q_vec(:,i) = q;
        i = i + 1;
        % Numerically integrate the robot configuration, since there is no
        % actual robot.
        q = q + T*u;

        % Update the title in order to provide a hint about when the simulation
        % will finish
        drawnow;
    end

    %% Runs the visualization until the user presses 'q'
    if visualize
        j = 1;
        while key ~= 'q'
            my_text = sprintf('Visualization number %d. Press ''q'' to quit.',j);
            title(my_text);
            for i = 1:size(q_vec,2)
                key = get(gcf,'CurrentCharacter');
                if key == 'q'
                    break;
                end
                plot(robot,q_vec(:,i)); 
                pause(0.005);
                drawnow;
            end
            title('Press any letter to visualize again or press ''q'' to quit.')
            pause;
            j = j + 1;
        end
    end

    if key == 'q'
        close all;
    end
end
