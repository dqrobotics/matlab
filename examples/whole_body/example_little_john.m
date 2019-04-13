% Simple example on how to perform whole body control using the Little John
% robot (http://macro.ppgee.ufmg.br/site-map/articles/14-front-end-articles/121-little-john)

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

function example_little_john(varargin)
    %default behavior is to visualize the trajectory
    visualize = 1; 
    if nargin == 1 && strcmp(varargin{1}, 'novisual')
        visualize = 0;
    end

    % Include the DQ name space to use i_ instead of DQ.i, etc.
    include_namespace_dq

    robot = LittleJohnRobot.kinematics();

    %% Initialize the configuration vector. 
    % All variables are zero except four. This is completely arbitrary.
    q = zeros(robot.get_dim_configuration_space(),1);

    %% Initialize controller parameters and control objective
    x = robot.fkm(q); % Initial end-effector pose

    % Define a rotation of pi/2 around the z-axis
    r = cos(pi/4) + k_*sin(pi/4);
    % Define a random translation
    p = 5*rand(1)*i_ + (15*rand(1)-7.5)*j_;

    % In order to define a decoupled transformation we use a CMI(3)
    % multiplication, also known as decompositional multiplication
    xd = (r + E_*(1/2)*p*r) .* x;


    %% Initalize the plot
    % Plot the global reference frame
    plot(DQ(1), 'scale', 0.2);
    hold on;
    plot(xd,'scale',0.2);

    % Plot the robot in the initial configuration
    plot(robot,q);
    %axis square;
    axis([-2,15,-15,15,0,0.7]);
    axis equal;
    hold on;
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
    while (key ~= 'q') & (norm(x_error) > 0.01)

        my_text = sprintf(['After the error becomes less than 0.01,'...
            ' the visualization will begin. Current error is %f'],norm(x_error));
        title(my_text)
        key = get(gcf,'CurrentCharacter');

        %% This is the part related to the robot motion control    
        x = robot.fkm(q);
        J = robot.pose_jacobian(q);
        N = haminus8(xd)*DQ.C8*J;
        x_error = vec8(x'*xd - 1);    
        % Use the damped least-square inverse in simple pseudoinverse-like kinematic
        % control. This is not the most appropriate controller for this robot, as 
        % the nonholonomy is not taken into account explicitly, but we just want to 
        % perform a simple simulation of Little John.
        u = -N'/(N*N' + 0.01*eye(size(N,1)))*gain*x_error;
        % Which ends here!

        % Store the current configuration to visualize it later
        q_vec(:,i) = q;
        i = i + 1;
        % Numerically integrate the robot configuration, since there is no
        % actual robot. Since the base inputs are the wheel velocities, we have
        % to map them back to the base configuration in order to perform the
        % integration        
        C = blkdiag(robot.get_chain{1}.constraint_jacobian(q(3)),eye(5));
        q = q + T*C*u;

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

