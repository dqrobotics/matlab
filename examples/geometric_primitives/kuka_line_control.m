% KUKA_LINE_CONTROL(zoom_value) runs a simple example with the KukaLwr4Robot. The
% goal is to make the robot end-effector's x-axis to be aligned with a
% random line generated in the space.
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

function kuka_line_control(varargin)
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

    %% Define a random line
    line_direction = normalize(DQ(rand(3,1)));
    line_point =  DQ(-0.3*ones(3,1)+ 0.6*rand(3,1));
    line_d = line_direction + E_ * cross(line_point, line_direction);

    %% Prepare the visualization
    figure;
    axis equal;
    grid off;
    view(-69,32);
    zoom(zoom_value);
    hold on;
    
    plot(kuka, q);
    % The reference is a black line
    plot(line_d,'line',3, 'color', 'k');
   
    
    %% Perform line control. 
    % The line passing through the end-effector x-axis will be aligned to
    % line_d.
    l_error = 1;
    l_effector = i_;
    while norm(l_error) > epsilon
        J = kuka.pose_jacobian(q);
        x = kuka.fkm(q);
        Jl = kuka.line_jacobian(J,x,l_effector);
        l = x*l_effector*x';
        
        l_error = vec8(line_d - l);
        q = q + pinv(Jl)*gain*l_error;
        plot(kuka, q);    
        drawnow;
    end
end




