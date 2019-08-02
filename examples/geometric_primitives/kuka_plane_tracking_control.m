% kuka_plane_tracking_control(total_time) runs an example where a KUKA LWR
% 4 has a plane attached to its end-effector and the goal is to align this
% plane to a time-varying plane in the workspace. total_time is the total
% time, in seconds (simulation time) for the time-varying plane trajectory;
% if the parameter is omitted, total_time = 30.
%
% This example uses a classic pseudoinverse-based controller with a
% feedforward term.

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

function kuka_plane_tracking_control(varargin)
    if nargin == 0
        total_time = 30;
    elseif nargin == 1
        total_time = varargin{1};
    else
        error('Usage kuka_lwr4_with_kuka_youbot(total_simulation_time)')
    end
    % include the namespace in order to use i_ instead of DQ_i, etc.
    include_namespace_dq

    %Create a new DQ_kinematics object with KUKA LWR parameters
    lwr4 = KukaLwr4Robot.kinematics();
    lwr4.name = 'KUKA LWR4';
    
    lwr4_controller = DQ_PseudoinverseController(lwr4);
    lwr4_controller.set_control_objective(ControlObjective.Plane);
    lwr4_controller.set_gain(1);
    lwr4_controller.set_stability_threshold(0.001);
    lwr4_controller.set_primitive_to_effector(k_);
    T = 0.1;
    
    %Initial configuration (completely arbitrary)
    q =[0    0.3770    0.1257   -0.5655         0         0         0]';
    
   
    %% Prepare the visualization
    figure;
    axis equal;
    grid off;
    view(8,11);
    axis([-1, 1.5, -1, 1.5, 0, 1.5]);    
    hold on;
    
    plot(lwr4, q, 'nojoints');    
    % The reference is a yellow plane
    handle_ref = plot(compute_reference(0), 'plane', 3, 'color', 'yellow');
    % We'll align a plane normal to the end-effector z-axis, passing
    % through the end-effector frame origin, with the desired yellow plane.
    handle_plane = plot(Adsharp(lwr4.fkm(q), k_),'plane',3,...
        'color','magenta');
    
    %% Perform plane control. 
    t = 0;   
    while t <= total_time
        
      
        [plane_d, feedforward] = compute_reference(t);
        u = lwr4_controller.compute_tracking_control_signal(q, ...
            vec8(plane_d), vec8(feedforward));
        q = q + T*u;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%% eye candy %%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % plot the lw4 robot
        plot(lwr4, q);    
        % plot the plane attached to the end-effector
        plane = Adsharp(lwr4.fkm(q), k_);
        handle_plane = plot(plane,'erase',handle_plane,'plane',1, 'color',...
            'magenta', 'location', translation(lwr4.fkm(q)));  
        
        % plot the time-varying plane
        handle_ref = plot(plane_d, 'erase', handle_ref, 'plane', 3, ...
            'color', 'yellow');
        drawnow;
        
        % update simulation time
        t = t + T;
        title(sprintf('t = %0.2f of %d seconds',t,total_time));
    end
end


% compute the time-varying reference for the plane
function [plane, plane_dot] = compute_reference(t)
    include_namespace_dq;

    % parameters for the trajectory
    d0 = 0.7;
    amp_dist = 0.1;
    wd = 0.5;
    wn = 0.5;
    n0 = (i_ + 2*k_)/norm(i_ + 2*k_);
    
    % calculate the time-varying distance between the plane and the
    % inertial system
    d = d0 + amp_dist*cos(wd * t);
    
    % calculate its time derivative
    ddot = -amp_dist*sin(wd * t)*wd;

    % calculate the time-varing plane orientation with respect to the
    % inertial system
    phi = (pi/4)*sin(wn*t);
    r = cos(phi/2) + k_ * sin(phi/2);
    n = Ad(r,n0);
    
    % calculate its time derivative
    phidot = (pi/4)*cos(wn*t)*wn;
    rdot = 0.5*(-sin(phi/2) + k_*cos(phi/2))*phidot;
    ndot = rdot * n0 * r' + r * n0 * rdot';

    % return the time-varying plane trajectory and its time derivative
    plane = n + E_ * d;    
    plane_dot = ndot + E_ * ddot;    
end




