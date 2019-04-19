% KUKA_PLANE_CONTROL(zoom_value) runs a simple example with the KukaLwr4Robot. 
% The goal is to align a plane perpendicular to the robot end-effector's z-axis, 
% passing through the end-effector frame origin, with a random plane 
% generated in the space.
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

function kuka_plane_control(varargin)
    if nargin == 0
        zoom_value = 1;
    elseif nargin == 1
        zoom_value = varargin{1};
    else
        error('Usage kuka_line_control(zoom_value)')
    end
    % include the namespace in order to use i_ instead of DQ_i, etc.
    include_namespace_dq

    %Create a new DQ_kinematics object with KUKA LWR parameters
    kuka = KukaLwr4Robot.kinematics();
    kuka.name = 'KUKA';

    %Initial configuration (completely arbitrary)
    q =[0    0.3770    0.1257   -0.5655         0         0         0]';
    
    %% Controller parameters
    % The error must be bellow this value in order to stop the robot
    epsilon = 0.001; 
    % Controller gains
    gain = 0.1;

    %% Define a random plane
    plane_normal = normalize(DQ(rand(3,1)));
    plane_point =  DQ(-0.3*ones(3,1)+ 0.6*rand(3,1));
    plane_d = -(plane_normal + E_ * dot(plane_point, plane_normal));

    %% Prepare the visualization
    figure;
    axis equal;
    grid off;
    view(42,9);
    zoom(zoom_value);
    hold on;
    
    plot(kuka, q, 'nojoints');    
    % The reference is a yellow plane
    plot(plane_d, 'plane', 3, 'color', 'yellow');
    % We'll align a plane normal to the end-effector z-axis, passing
    % through the end-effector frame origin, with the desired yellow plane.
    plane_effector = k_;
    handle_plane = plot(Adsharp(kuka.fkm(q), plane_effector),'plane',3,...
        'color','magenta');
    
    %% Perform plane control. 
 
    plane_error = 1;
    while norm(plane_error) > epsilon
        J = kuka.pose_jacobian(q);
        x = kuka.fkm(q);
        Jl = kuka.plane_jacobian(J,x,plane_effector);
        
        % The plane_effector is expressed in the reference frame 
        plane = Adsharp(x, plane_effector);
        
        % Just eye candy to show the magenta plane
        handle_plane = plot(plane,'erase',handle_plane,'plane',3, 'color',...
            'magenta');
        
        % The controller is just a simple kinematic one.
        plane_error = vec8(plane_d - plane);
        q = q + pinv(Jl)*gain*plane_error;
        plot(kuka, q);    
        drawnow;
    end
end




