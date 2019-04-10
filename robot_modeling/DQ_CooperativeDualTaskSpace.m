% CLASS DQ_cdts
% Usage: cdts = DQ_cdts(robot1,robot2), where:
% - robot1 and robot2 are objects of type DQ_kinematics;
% 
% By using this class, the cooperative system is described by the
% cooperative variables absolute_pose and relative_pose, and their respective Jacobians absolute_pose_jacobian and relative_pose_jacobian
%
% Type DQ_cdts.(method or property) for specific help.
% Ex.: help DQ_cdts.absolute_pose
%
% METHODS:
%       absolute_pose
%       relative_pose
%       absolute_pose_jacobian
%       relative_pose_jacobian
%       pose1
%       pose2
%

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

classdef DQ_CooperativeDualTaskSpace
    properties
        robot1, robot2;
    end
    
    methods
        function obj = DQ_CooperativeDualTaskSpace(robot1, robot2)
            if ~isa(robot1,'DQ_kinematics') || ~isa(robot2,'DQ_kinematics')
                error(['The DQ_cdts class must be initialized with the '...
                       'kinematics information of each robot']);
            else
                obj.robot1 = robot1;
                obj.robot2 = robot2;
            end
        end
        
        function x = pose1(obj,theta)
            % x = pose1(theta) returns the pose of the first end-effector,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            theta1=theta(1:obj.robot1.n_links);
            x = obj.robot1.fkm(theta1);
        end
        
        function x = pose2(obj,theta)
            % x = pose2(theta) returns the pose of the second end-effector,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]           
            theta2=theta(obj.robot1.n_links+1:end);
            x = obj.robot2.fkm(theta2);
        end
        
        function J = jacobian1(obj,theta)
            % J = jacobian1(theta) returns the jacobian of the first arm,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            theta1=theta(1:obj.robot1.n_links);
            J = obj.robot1.pose_jacobian(theta1);
        end
        
        function J = jacobian2(obj,theta)
            % J = jacobian2(theta) returns the jacobian of the second arm,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            theta2=theta(obj.robot1.n_links+1:end);
            J = obj.robot2.pose_jacobian(theta2);
        end
        
        function x = relative_pose(obj,theta)
            % x = relative_pose(theta) returns the relative dual position,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
                       
            x = obj.pose2(theta)'*obj.pose1(theta);
        end
        
        function x = absolute_pose(obj,theta)
            % x = absolute_pose(theta) returns the absolute dual position,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]            
            x = obj.pose2(theta)*(obj.relative_pose(theta)^0.5);
        end
        
        function jac = relative_pose_jacobian(obj,theta)
            % jac = relative_pose_jacobian(theta) returns the relative Jacobian,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            jac = [hamiplus8(obj.pose2(theta)')*obj.jacobian1(theta), ...
                   haminus8(obj.pose1(theta))*DQ.C8*obj.jacobian2(theta)];
        end
        
        function jac = absolute_pose_jacobian(obj, theta)
            % jac = absolute_pose_jacobian(theta) returns the absolute Jacobian,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            x2 = obj.pose2(theta);
            
            jacob2 = obj.jacobian2(theta);            
            jacobr = obj.relative_pose_jacobian(theta);
            xr=obj.relative_pose(theta);
            
            jacob_r2 = 0.5*haminus4(xr.P'*(xr.P)^0.5)*jacobr(1:4,:);
            jacobp_r = DQ_kinematics.translation_jacobian(jacobr,xr);
            
            jacob_xr_2 = [jacob_r2;  0.25*(haminus4(xr.P^0.5)*jacobp_r + ...
                                     hamiplus4(translation(xr))*jacob_r2)];
            
            jac = haminus8(xr^0.5)*[zeros(8,obj.robot1.n_links),jacob2] + ...
                  hamiplus8(x2)*jacob_xr_2;
        end
    end
end