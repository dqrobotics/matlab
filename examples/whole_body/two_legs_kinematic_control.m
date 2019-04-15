% TWO_LEGS_KINEMATIC_CONTROL(zoom_value) runs an example on how to build two
% legs from two KUKA LWR4 Robots, step by step. In order use the whole chain, 
% the serialization process takes into account reverse chains. 
%
% The input 'zoom_value' determines the zoom used in the visualization. If no 
% parameter is passed to the function, 'zoom_value' equals 1.

% (C) Copyright 2011-2019 DQ Robotics Developers
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

% function two_legs_example(varargin)
%     %default behavior is to visualize the trajectory
%     visualize = 1; 
%     if nargin == 1 && strcmp(varargin{1}, 'novisual')
%         visualize = 0;
%     end

function two_legs_kinematic_control(varargin)
    if nargin == 0
        zoom_value = 1;
    elseif nargin == 1
        zoom_value = varargin{1};
    else
        error('Usage two_legs_kinematic_control(zoom_value)')
    end
    
    % Include the DQ name space to use i_ instead of DQ.i, etc.
    include_namespace_dq
    
    % The legs are composed of two KUKA LWR4 robots coupled together    
    right_leg = KukaLwr4Robot.kinematics();
    right_leg.name = 'Right leg';
    left_leg = KukaLwr4Robot.kinematics();
    left_leg.name = 'Left leg';
    
    % Let's first assemble the scene    
    figure;
    view(0,20);
    zoom(zoom_value);
    hold on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    % First we draw a horizontal plane just 4 mm deep into the floor
    plane = k_ - E_*0.004;
    plot(plane,'plane',4,'color','y');
    
    %% Let's assemble the robot.
    % This is just for the sake of example. Ideally, we would create a
    % dedicated class, such as DQ_LeggedRobot to do all of the low-level
    % assembly.
    
    % The left foot will be located 20 cm to the left, along the x axis,
    % but with the z-axis pointing downwards.
    left_foot = (1 + E_ * 0.5 * (-0.2*i_ - j_)) * i_;
    % The right foot will be located 20 cm to the right, along the x axis,
    % but with the z-axis pointing downwards.
    right_foot = (1 + E_ * 0.5 * (0.2*i_ - j_)) * i_;
    
    % Physically place the left leg in the workspace
    x_0_to_left_effector = left_leg.fkm(zeros(7,1));
    new_base_frame = left_foot .* T(x_0_to_left_effector);
    left_leg.set_base_frame(new_base_frame);
    % Physically place the right leg in the workspace
    x_0_to_right_effector = right_leg.fkm(zeros(7,1));
    new_base_frame = right_foot .* T(x_0_to_right_effector);
    right_leg.set_base_frame(new_base_frame);
    
    %% Initializes the serially coupled kinematic chains
    % From the left leg towards the right leg
    left_to_right = DQ_WholeBody(left_leg, 'reversed');
    % Determines the rigid transformation between the base frames of the
    % two legs
    base_left_to_base_right = left_leg.base_frame'*right_leg.base_frame;
    % Add this constant transformation to the chain
    left_to_right.add(base_left_to_base_right);    
    % Finally, add the right frame
    left_to_right.add(right_leg);
    
    % We do the same thing, but from the right leg towards the left leg
    right_to_left = DQ_WholeBody(right_leg, 'reversed');
    % Determines the rigid transformation between the base frames of the
    % two legs
    base_right_to_base_left = base_left_to_base_right';
    % Add this constant transformation to the chain
    right_to_left.add(base_right_to_base_left);    
    % Finally, add the left leg
    right_to_left.add(left_leg);
    
    
    %% All set. Now it's time to show everything
    %% Initializing the keyboard 'stroke capturing system' with a value
    % different from  'q'. This is to offer the possibility for the user to
    % quit the program by pressing 'q'.
    key = '@';
    set(gcf,'CurrentCharacter',key);   
    q = zeros(14,1);
    plot(left_to_right,q);
   
    % Let's start this simulation with the right foot. (Pun intended!)
    right_step = true;
    step_base_frame = left_foot;
    
    % Controller parameters
    integration_step = 0.001;
    gain = 100;
    
    % We'll use this frame to show where the robot should step on.
    h_plot = plot(right_foot, 'scale', 0.1);
   
    % Let's roll (or should I say let's run?)
    % The robot will walk four steps
    for step = 1:4
        step_size = 1 + E_* 1/2 * step * 0.6 * j_;
        
        if right_step
            robot = left_to_right;
            xd = right_foot .* step_size;
            title('Step with the right foot');
        else %left foot
            robot = right_to_left;
            xd = left_foot .* step_size;
            title('Step with the left foot');
        end
        
        robot.set_base_frame(step_base_frame);
        robot.set_reference_frame(step_base_frame);
        
        h_plot = plot(xd, 'erase', h_plot, 'scale', 0.1);
  
        x_error = 1;      
        while key ~= 'q' && norm(x_error) > 0.01           
            key = get(gcf,'CurrentCharacter');
            
            % Retrieve the FKM and pose Jacobian
            x = robot.fkm(q);
            J = robot.pose_jacobian(q); 
            
            % Calculate the error and the control input
            x_error = vec8(x-xd);       
            u = -pinv(J)*gain*x_error;
            
            % Do a numerical integration because we are not simulating an
            % "actual" robot
            q = q + integration_step*u;  
            plot(robot,q);
            drawnow;
        end
        step_base_frame = robot.fkm(q);
        % Let's change the foot
        right_step = ~right_step;
        % If we change the sequence of the kinematic chain, we have to
        % change the configuration vector accordingly.
        q = [q(8:end);q(1:7)];
        if key == 'q'
            close all;
            return;
        end
    end  
end
