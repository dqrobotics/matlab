% CLASS DQ_KinematicControl implement several (first order) kinematic controllers 
% for a broad class of robots (i.e., controllers based on the model vec8(xdot) = J * qdot)
%   
% Ways of defining a new controller:
%   controller = DQ_KinematicController(robot), where robot is an object of
%   type DQ_kinematics
%
% Controllers (for more information, type help DQ_KinematicController.<CONTROLLER_NAME>):
%       pseudoinverse_pose_controller
%
% All controllers are based on the following data structure:
        % Minimal structure (i.e., all controllers must have the following
        % information):
        %     controller_variables.q: current robot configuration;
        %     controller_variables.K: current gain matrix;
        %     controller_variables.xd: current desired end-effector pose;
        

% (C) Copyright 2017 DQ Robotics Developers
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
classdef DQ_KinematicController
% Information for developers
% We use some standard definitions for variables:
% u : control input
% J : Jacobian matrix
% x : current end-effector pose
% xd : desired end-effector pose
% q : current robot configuration
% qd : desired robot configuration
% e : error

    properties
        robot; % DQ_kinematics object     
    end
    
    methods
        function obj = DQ_KinematicController(robot)
            if nargin ~= 1
                error_message = ['There is no associated robot. Please use the' ...
                                'following syntax controller = DQ_KinematicController(robot),'...
                                'where robot is an object of type DQ_kinematics'];
                error(error_message);
            else
                obj.robot = robot;
            end
        end
        
        function [u, varargout] = pseudoinverse_pose_controller(obj,controller_variables) 
        % pseudoinverse_pose_controller    Classic pose controller based on the 
        % pseudoinverse of the Jacobian matrix and the Euclidean error: 
        % u = pinv(J)*K*vec8(xd-x).
        %
        % [u,e] = pseudoinverse_pose_controller(controller_variables)
        % returns the control input u (joint velocities) and the current error
        % e = vec(xd - x), where xd is the desired pose and x is the current one.
        %
        % u = = pseudoinverse_pose_controller(controller_variables) returns
        % only the control input u.
        %
        % This controller needs only the minimal structure for controller
        % variables (see help DQ_KinematicController)
        
            J = obj.robot.jacobian(controller_variables.q);
            x = obj.robot.fkm(controller_variables.q);
            e = vec8(controller_variables.xd - x);
            u = pinv(J)*controller_variables.K*e; 
            if nargout == 2
                varargout{1} = e;
            end
        end
        
        function [u, varargout] = damped_pseudoinverse_pose_controller(obj,controller_variables)
        % damped_pseudoinverse_pose_controller    Pose controller based on the 
        % damped pseudoinverse of the Jacobian matrix and the Euclidean error: 
        % u = dinv(J)*K*vec8(xd-x), where 
        % dinv(J) = J'*inv(J*J'+lambda^2*I), if n > 6 (redundant system)
        % dinv(J) = inv(J'*J + lambda^2*I)*J' if n <= 6 (underactuated or completely actuated)
        % 
        % [u,e] = damped_pseudoinverse_pose_controller(controller_variables)
        % returns the control input u (joint velocities) and the current error
        % e = vec(xd - x), where xd is the desired pose and x is the current one.
        %
        % u = = pseudoinverse_pose_controller(controller_variables) returns
        % only the control input u.
        %
        % This controller needs the minimal structure for controller
        % variables (see help DQ_KinematicController) plus the damping
        % factor.
            J = obj.robot.jacobian(controller_variables.q);
            x = obj.robot.fkm(controller_variables.q);
            e = vec8(controller_variables.xd - x);
            
            %Calculates the damped pseudoinverse
            dof = length(obj.robot.q);
            if dof <= 6
                dinv_J = (J'*J + controller_variables.lambda^2*eye(dof))\J';
            else
                dinv_J = J'/(J*J'+controller_variables.lambda^2*eye(8));
            end
                
            u = dinv_J*controller_variables.K*e;
            if nargout == 2
                varargout{1} = e;
            end
        end
    end
end

