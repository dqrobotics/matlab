% Basic implementation of a Differential Drive Mobile robot.
% 
% Usage: robot = DQ_DifferentialDriveRobot(wheel_radius,distance_between_wheels), 
% where both wheel_radius and distance_between_wheels are given in meters. 
%
% DQ_DifferentialDriveRobot Properties:
%       wheel_radius - radius of the wheels.
%       distance_between_wheels - distance between the wheels.
%
% DQ_DifferentialDriveRobot Methods (Concrete):
%       constraint_jacobian - Compute the Jacobian that relates the wheels velocities to the configuration velocities.
%       pose_jacobian (overloaded) - Compute the pose Jacobian by taking into account the nonholonomic constraints.
%
% See also DQ_HolonomicBase.


% (C) Copyright 2011-2025 DQ Robotics Developers
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
%    1.  Bruno Vihena Adorno - adorno@ieee.org
%
%    2. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
%        - Added the method pose_jacobian_derivative(). 
%          However, the method is not implemented yet.

classdef DQ_DifferentialDriveRobot < DQ_HolonomicBase
    
    properties (Access = protected)
        wheel_radius;
        distance_between_wheels; 
    end
    
    methods
        function obj = DQ_DifferentialDriveRobot(wheel_radius,...
                distance_between_wheels)
            obj.wheel_radius = wheel_radius;
            obj.distance_between_wheels = distance_between_wheels;
            obj.base_diameter = obj.distance_between_wheels;
        end
        
        function J = constraint_jacobian(obj,phi)
            % J = constraint_jacobian(phi) returns the Jacobian matrix that
            % satisfies [x_dot, y_dot, phi_dot]' = J*[wr,wl]', where wr and
            % wl are the angular velocities of the right and left wheels,
            % respectively.
            % phi is the robot orientation angle
            r = obj.wheel_radius;
            l = obj.distance_between_wheels;
            c = cos(phi);
            s = sin(phi);
            
            J = [(r/2)*c, (r/2)*c;
                 (r/2)*s, (r/2)*s;
                  r/l, -r/l
                ];
        end
                
        function J = pose_jacobian(obj,q)
            % J = pose_jacobian(q) returns the Jacobian matrix that
            % satisfiex vec8(h_dot) = J*[wr,wl]', where h_dot is the time
            % derivative of the unit dual quaternion that represent the
            % mobile base pose and  wr and wl are the angular velocities of the
            % right and left wheels, respectively. 
            % q = [x,y,phi] is the robot configuration
            J_holonomic = pose_jacobian@DQ_HolonomicBase(obj,q);
            J = J_holonomic*obj.constraint_jacobian(q(3));
        end

        function J_dot = pose_jacobian_derivative(obj, q, q_dot)
        % J_dot = pose_jacobian_derivative(q, q_dot) returns the time
        % derivative of the Jacobian matrix that satisfies 
        % vec8(h_dot_dot) = J_dot*[wr,wl]' + J*[wr_dot,wl_dot]', where h_dot is the time
        % derivative of the unit dual quaternion that represent the
        % mobile base pose and  wr and wl are the angular velocities of the
        % right and left wheels, respectively. 
        % q = [x,y,phi] is the robot configuration

            error('pose_jacobian_derivative is not implemented yet');
        end
    end
end