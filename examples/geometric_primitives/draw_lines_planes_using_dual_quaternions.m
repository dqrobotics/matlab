% This example shows how to use the plot functions from the DQ class in
% order to draw Plucker lines and planes using dual quaternion
% representation. For more information on the theory behind this
% representation, please see Sections 2.4 and 2.5?from 
% "Adorno, B. V. (2017). Robot Kinematic Modeling and Control Based on Dual 
% Quaternion Algebra -- Part I: Fundamentals," which can be downloaded at 
% https://hal.archives-ouvertes.fr/hal-01478225v1

% (C) Copyright 2015 DQ Robotics Developers
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
%     Bruno Vihena Adorno - adorno@ufmg.br

function draw_lines_planes_using_dual_quaternions()

    include_namespace_dq

    % Rotation of pi around the y-axis;
    r = j_;
    % Translation (1,1,1)
    p = i_ + j_ + k_;

    % Initial location of the moving frame
    x = r + E_*0.5*p*r;

    % Plot the reference frame
    plot(DQ(1), 'name', '$\mathcal{F}_0$');
    hold on;
    axis([-4, 4, -4, 4, -4, 4])
    handle_x = plot(x, 'name', '$\mathcal{F}_1$');

    % Define a plane, with respect to the local frame, perpendicular to the z-axis 
    % with 2 units of distance from the xy-plane.
    plane = k_ + E_* 2;
    % Plot the plane and get its handle. The handle will be used to move the
    % plane in subsequent plots.
    handle_plane = plot(x.'*plane*x','plane',5, 'color', 'g');

    % Draw four lines, with respect to the local frame, perpendicular to and 2 units 
    % of distance from the z-axis and with 1 unit of distance from the xy-plane. 
    l1 = j_ + E_*cross(-2*i_ + k_, j_); 
    l2 = j_ + E_*cross( 2*i_ + k_, j_);
    l3 = i_ + E_*cross( 2*j_ + k_, i_);
    l4 = i_ + E_*cross(-2*j_ + k_, i_);

    % Plot the four lines and get their handles. The handles will be used to
    % move the lines in subsequent plots.
    handle_l1 = plot(x*l1*x','line', 8);
    handle_l2 = plot(x*l2*x','line', 8);
    handle_l3 = plot(x*l3*x','line', 8);
    handle_l4 = plot(x*l4*x','line', 8);

    % Move the coordinate frame x towards the reference frame using screw
    % linear interpolation. All geometrical objects will be moved accordingly
    for ii = 1:-0.01:0
          y = x^ii;
          handle_x = plot(y, 'erase', handle_x, 'name', '$\mathcal{F}_1$');
          handle_l1 = plot(y*l1*y', 'erase', handle_l1, 'line',8);
          handle_l2 = plot(y*l2*y', 'erase', handle_l2, 'line',8);
          handle_l3 = plot(y*l3*y', 'erase', handle_l3, 'line',8);
          handle_l4 = plot(y*l4*y', 'erase', handle_l4, 'line',8);
          handle_plane = plot(y.'*plane*y', 'erase', handle_plane, 'plane', 5);
          pause(0.001);
    end
end


