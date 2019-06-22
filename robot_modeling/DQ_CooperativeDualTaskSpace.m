% Implement bimanual manipulation in terms of the cooperative dual taskspace framework.
%
% The cooperative system is described by the cooperative variables, 
% namely absolute_pose and relative_pose, and their respective Jacobians, 
% namely absolute_pose_jacobian and relative_pose_jacobian.
%
% Usage: cdts = DQ_CooperativeDualTaskSpace(robot1,robot2), where:
% - robot1 and robot2 are objects of type DQ_Kinematics;
% 
% DQ_CooperativeDualTaskSpace Properties:
%       robot1 - Any object from a class that inherits from DQ_Kinematics.
%       robot2 - Any object from a class that inherits from DQ_Kinematics.
%
% DQ_CooperativeDualTaskSpace Methods (Concrete):
%       absolute_pose - Compute the absolute pose.
%       absolute_pose_jacobian - Compute the Jacobian related to the time derivative of the absolute pose.
%       pose_jacobian1 - Return the pose Jacobian of the first robot.
%       pose_jacobian2 - Return the pose Jacobian of the second robot.
%       pose1 - Return the pose of the first end-effector.
%       pose2 - Return the pose of the second end-effector.
%       relative_pose - Compute the rigid transformation between the two end-effectors.
%       relative_pose_jacobian - Compute the Jacobian related to the time derivative of relative pose.
%

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

classdef DQ_CooperativeDualTaskSpace
    properties
        robot1, robot2;
    end
    
    methods
        function obj = DQ_CooperativeDualTaskSpace(robot1, robot2)
            if ~isa(robot1,'DQ_Kinematics') || ~isa(robot2,'DQ_Kinematics')
                error(['The DQ_CooperativeDualTaskSpace class must be '
                       'initialized with the kinematics information of each '...
                       'robot']);
            else
                obj.robot1 = robot1;
                obj.robot2 = robot2;
            end
        end
        
        function x = pose1(obj,q)
            % x = pose1(q) returns the pose of the first end-effector,
            % where 'q' is the joint position of the resultant system;
            % that is, q = [q1;q2]
            q1 = q(1:obj.robot1.get_dim_configuration_space);
            x = obj.robot1.fkm(q1);
        end
        
        function x = pose2(obj,q)
            % x = pose2(q) returns the pose of the second end-effector,
            % where 'q' is the joint position of the resultant system;
            % that is, q = [q1;q2]           
            q2 = q(obj.robot1.get_dim_configuration_space+1:end);
            x = obj.robot2.fkm(q2);
        end
        
        function J = pose_jacobian1(obj,q)
            % J = pose_jacobian1(q) returns the jacobian of the first arm,
            % where q is the joint position of the resultant system;
            % that is, q = [q1;q2]
            q1 = q(1:obj.robot1.get_dim_configuration_space);
            J = obj.robot1.pose_jacobian(q1);
        end
        
        function J = pose_jacobian2(obj,q)
            % J = pose_jacobian2(q) returns the jacobian of the second arm,
            % where q is the joint position of the resultant system;
            % that is, q = [q1;q2]
            q2 = q(obj.robot1.get_dim_configuration_space+1:end);
            J = obj.robot2.pose_jacobian(q2);
        end
        
        function x = relative_pose(obj,q)
            % x = relative_pose(q) returns the relative pose,
            % where q is the joint position of the resultant system;
            % that is, q = [q1;q2]
                       
            x = obj.pose2(q)'*obj.pose1(q);
        end
        
        function x = absolute_pose(obj,q)
            % x = absolute_pose(q) returns the absolute pose; that is, the
            % pose of a frame located in between the two end-effectors,
            % where 'q' is the configuration vector of the resultant system;
            % that is, q = [q1;q2]            
            x = obj.pose2(q)*(obj.relative_pose(q)^0.5);
        end
        
        function Jr = relative_pose_jacobian(obj,q)
            % jac = relative_pose_jacobian(q) returns the relative Jacobian,
            % where 'q' is the joint position of the resultant system;
            % that is, q = [q1;q2]
            Jr = [hamiplus8(obj.pose2(q)')*obj.pose_jacobian1(q), ...
                   haminus8(obj.pose1(q))*DQ.C8*obj.pose_jacobian2(q)];
        end
        
        function Ja = absolute_pose_jacobian(obj, q)
            % jac = absolute_pose_jacobian(q) returns the absolute Jacobian,
            % where 'q' is the joint position of the resultant system;
            % that is, q = [q1;q2]
            x2 = obj.pose2(q);
            
            jacob2 = obj.pose_jacobian2(q);            
            jacobr = obj.relative_pose_jacobian(q);
            xr=obj.relative_pose(q);
            
            jacob_r2 = 0.5*haminus4(xr.P'*(xr.P)^0.5)*jacobr(1:4,:);
            jacobp_r = DQ_kinematics.translation_jacobian(jacobr,xr);
            
            jacob_xr_2 = [jacob_r2;  0.25*(haminus4(xr.P^0.5)*jacobp_r + ...
                                     hamiplus4(translation(xr))*jacob_r2)];
            
            Ja = haminus8(xr^0.5) * ...
                [zeros(8,obj.robot1.get_dim_configuration_space),jacob2] + ...
                  hamiplus8(x2)*jacob_xr_2;
        end
    end
end