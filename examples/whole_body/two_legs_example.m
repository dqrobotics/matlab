% Example on how to build two legs from two KUKA LWR4 Robots, step by step. In
% order use the whole chain, the serialization process takes into account
% reverse chains. 
%
% Usage: TWO_LEGS_EXAMPLE(zoom_value) runs the example with zoom given by
% 'zoom_value'. If no parameter is passed to the function, 'zoom_value'
% equals 1.
%
% General instructions:
% 1) After the program starts, press the up-arrow and down-arrow keys to
% increase or decrease the joint angle, respectively. 
% 2) The joint can be changed by pressing the number keys (1 for the first joint, 2
% for the second and so on). 
% 3) Press 's' to change between the original and sequential modes. The
% former uses the ordering of the original kinematic chains. For instance,
% if the first chain has 7 DOF and is reversed, in original mode joint 1 
% corresponds to the seventh joint. In the sequential mode, a more
% intuititve approach is used and joint 1 corresponds to the first joint of
% the whole-body kinematic chain.
  

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

function two_legs_example(varargin)
    if nargin == 0
        zoom_value = 1;
    elseif nargin == 1
        zoom_value = varargin{1};
    else
        error('Usage two_legs_example(zoom_value)')
    end

    % Include the DQ name space to use i_ instead of DQ.i, etc.
    include_namespace_dq
    
    % The legs are composed of two KUKA LWR4 robots coupled together    
    right_leg = KukaLwr4Robot.kinematics();
    right_leg.name = 'Right leg';
    left_leg = KukaLwr4Robot.kinematics();
    left_leg.name = 'Left leg';
    
    % Let's first assemble the scene
    % Key-press events are handled by the function keypress.
    % They will be used to change the joint angles of both robots
    fig_handle = figure('KeyPressFcn', @keypress);
    view(18,51);
    zoom(zoom_value);
    hold on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    % First we draw a horizontal plane just 4 mm deep into the floor
    plane = k_ - E_*0.004;
    plot(plane,'plane',4,'color','y');
    % Now we plot the global reference frame (a.k.a, the identity)
    plot(DQ(1), 'name', 'Global', 'scale', 0.1);
    % The left foot will be located 20 cm to the left, along the x axis,
    % but with the z-axis pointing downwards.
    left_foot = (1 - E_ * 0.5 * 0.2 * i_) * i_;
    % We don't want to plot the left-foot frame, only its name
    plot(left_foot, 'name', 'Left foot', 'scale', 0);
    % The right foot will be located 20 cm to the right, along the x axis,
    % but with the z-axis pointing downwards.
    right_foot = (1 + E_ * 0.5 * 0.2 * i_) * i_;
    plot(right_foot, 'name', 'Right foot', 'scale', 0);
    
    % Physically place the left leg in the workspace
    x_0_to_left_effector = left_leg.fkm(zeros(7,1));
    new_base_frame = left_foot .* T(x_0_to_left_effector);
    left_leg.set_base_frame(new_base_frame);
    % Physically place the right leg in the workspace
    x_0_to_right_effector = right_leg.fkm(zeros(7,1));
    new_base_frame = right_foot .* T(x_0_to_right_effector);
    right_leg.set_base_frame(new_base_frame);
    
    %% Initializes the robot serially coupled kinematic chain from the mobile base
    left_to_right = DQ_WholeBody(left_leg, 'reversed');
    % The base frame is the left foot. The reference frame, also located at
    % the left foot, provides the transformation with respect to the global
    % frame. Therefore, any element in the chain will be given with respect
    % to the global frame.
    left_to_right.set_reference_frame(left_foot);
    left_to_right.set_base_frame(left_foot);
    
    % Determines the rigid transformation between the base frames of the
    % two legs
    base_left_to_base_right = left_leg.base_frame'*right_leg.base_frame;
    % Add this constant transformation to the chain
    left_to_right.add(base_left_to_base_right);    
    % Finally, add the right frame
    left_to_right.add(right_leg);
    
    %% All set. Now it's time to show everything
    q = zeros(14,1);
    plot(left_to_right,q);
    
    %% The struct S is shared between the main function and the keypress
    % function.
    
    % increment to be applied to a particular joint
    S.inc = 0; 
    % The program quits when this variable equals '@'
    S.quit = '@'; 
    % Stores the joint that'll be changed
    S.joint = 1; 
    % The configuration vector follows the ordering of the original
    % kinematic chain.
    S.sequential = false;  
    %Store the struct in the figure
    guidata(fig_handle,S);
    
    % Let's plot a text just above the legs.
    frame_text = left_to_right.fkm(q,1);
    tc = vec3(translation(frame_text) + k_*0.3);
    update_text = sprintf(['Press up and down arrows \n'...
        ' to change value of joint %d. \nCurrent mode: %s. \nPress ''s'' to'...
        ' change modes.'], S.joint, 'original');
    text_handle = text(tc(1),tc(2),tc(3),update_text,'FontSize', 20);
    
    
    %% Robot teleoperation with the keyboard
    while S.quit ~= 'q'
        % Get the information updated by the keypress() function
        S = guidata(fig_handle); 
        if S.sequential == true
            text_sequential = 'sequential';
        else
            text_sequential = 'original';
        end
        
        update_text = sprintf(['Press up and down arrows \n'...
        ' to change value of joint %d. \nCurrent mode: %s. \nPress ''s'' to'...
        ' change modes.'], S.joint, text_sequential);
        set(text_handle,'String',update_text);
                 
        % Update the joints
        q(S.joint) = q(S.joint) + S.inc;
        S.inc = 0;
        % Reset the increment value that is stored in the figure
        guidata(fig_handle,S);
        if S.sequential
            plot(left_to_right, left_to_right.sequential(q));
        else
            plot(left_to_right, q);
        end
        drawnow;
    end
    close all;
end

% keypress() handles the keyboard events
function keypress(H,E)
    S = guidata(H);
    
    %Self-explanatory state machine.
    switch E.Key
        case 'uparrow'            
            S.inc = 45*pi/180;
        case 'downarrow'
            S.inc = -45*pi/180;
        case 'q'
            S.quit = 'q';
        case 's'
            S.sequential = ~S.sequential;
        otherwise
            % Determines the joint to be changed
            number = str2num(E.Character);
            if number
                S.joint = number;
            end
    end
    
    % Store the updated control inputs and the program status (S.quit)
    guidata(H,S);
end
