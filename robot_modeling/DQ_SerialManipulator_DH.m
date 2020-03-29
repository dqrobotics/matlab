% Concrete class that extends the DQ_SerialManipulator using the
% Denavit-Hartenberg parameters (DH)
%
% Usage: robot = DQ_SerialManipulator_DH(A)
% - 'A' is a 4 x n matrix containing the Denavit-Hartenberg parameters
%   (n is the number of links)
%    A = [theta1 ... thetan;
%            d1  ...   dn;
%            a1  ...   an;
%         alpha1 ... alphan;
%         type1  ... typen]
% where type is the actuation type, either DQ_SerialManipulator_DH.JOINT_ROTATIONAL
% or DQ_SerialManipulator_DH.JOINT_PRISMATIC
% - The only accepted convention in this subclass is the 'standard' DH
% convention.
%
% If the joint is of type JOINT_ROTATIONAL, then the first row of A will
% have the joint offsets. If the joint is of type JOINT_PRISMATIC, then the
% second row of A will have the joints offsets.
%
% DQ_SerialManipulator_DH Methods (Concrete):
%       get_dim_configuration_space - Return the dimension of the configuration space.
%       fkm - Compute the forward kinematics while taking into account base and end-effector's rigid transformations.
%       plot - Plots the serial manipulator.
%       pose_jacobian - Compute the pose Jacobian while taking into account base's and end-effector's rigid transformations.
%       pose_jacobian_derivative - Compute the time derivative of the pose Jacobian.
%       raw_fkm - Compute the FKM without taking into account base's and end-effector's rigid transformations.
%       raw_pose_jacobian - Compute the pose Jacobian without taking into account base's and end-effector's rigid transformations.
%       set_effector - Set an arbitrary end-effector rigid transformation with respect to the last frame in the kinematic chain.
% See also DQ_SerialManipulator.

% (C) Copyright 2020 DQ Robotics Developers
%
% This file is part of DQ Robotics.
%
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published
%     by the Free Software Foundation, either version 3 of the License, or
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
%     Murilo M. Marinho - murilo@nml.t.u-tokyo.ac.jp

classdef DQ_SerialManipulator_DH < DQ_SerialManipulator
    properties
        type
    end
    
    properties (Constant)
        % Joints that can be actuated
        % Rotational joint
        JOINT_ROTATIONAL = 1;
        % Prismatic joint
        JOINT_PRISMATIC = 2;
    end
    
    methods
        function obj = DQ_SerialManipulator_DH(A,convention)
            % These are initialized in the constructor of
            % DQ_SerialManipulator
            %obj.convention = convention;
            %obj.n_links = size(A,2);
            %obj.theta = A(1,:);
            %obj.d = A(2,:);
            %obj.a = A(3,:);
            %obj.alpha = A(4,:);
            %obj.dummy = A(5,:);
            
            obj = obj@DQ_SerialManipulator(A,convention);
            if nargin == 0
                error('Input: matrix whose columns contain the DH parameters')
            end
            
            if(size(A,1) ~= 5)
                error('Input: Invalid DH matrix. It should have 5 rows.')
            end
            
            % Remove dummy joints 
            obj.dummy = zeros(1,obj.n_links);
            obj.n_dummy = 0;
            
            % Add type
            obj.type = A(5,:);
            
        end
        
        function x = raw_fkm(obj,q,to_ith_link)
            %   RAW_FKM(q) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   last joint (the displacements due to the base and the effector
            %   are not taken into account).
            %
            %   'q' is the vector of joint variables
            %   'to_ith_link' defines until which link the raw_fkm will be
            %   calculated.
            %
            %   This is an auxiliary function to be used mainly with the
            %   Jacobian function.
            if nargin == 3
                n = to_ith_link;
            else
                n = obj.n_links;
            end
            
            x = DQ(1);
            
            for i=1:n
                x = x*dh2dq(obj,q(i),i);
            end
        end
        
        function x = fkm(obj,q,to_ith_link)
            %   FKM(q) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   end-effector pose. This function takes into account the
            %   displacement due to the base's and effector's poses.
            %
            %   'q' is the vector of joint variables
            %   'to_ith_link' defines up to which link the fkm will be
            %   calculated. If to_ith_link corresponds to the last link,
            %   the method DOES NOT take into account the transformation
            %   given by set_effector. If you want to take into account
            %   that transformation, use FKM(q) instead.
            
            if nargin == 3
                x = obj.reference_frame*obj.raw_fkm(q, to_ith_link); %Takes into account the base displacement
            else
                x = obj.reference_frame*obj.raw_fkm(q)*obj.effector;
            end
        end
        
        function dq = dh2dq(obj,q,ith)
            %   For a given link's Extended DH parameters, calculate the correspondent dual
            %   quaternion
            %   Usage: dq = dh2dq(q,ith), where
            %          q: joint value
            %          ith: link number
            
            if nargin ~= 3
                error('Wrong number of arguments. The parameters are joint value and the correspondent link')
            end
            
            theta_ = obj.theta(ith);
            d_ = obj.d(ith);
            a_ = obj.a(ith);
            alpha_ = obj.alpha(ith);
            
            % If joint is rotational
            if obj.type(ith) == obj.JOINT_ROTATIONAL
                h1 = cos((theta_+q)/2.0)+DQ.k*sin((theta_+q)/2.0);
                h2 = 1 + DQ.E*0.5*d_*DQ.k;
                % If joint is prismatic
            else
                h1 = cos(theta_/2.0)+DQ.k*sin(theta_/2.0);
                h2 = 1 + DQ.E*0.5*(d_+q)*DQ.k;
            end
            h3 = 1 + DQ.E*0.5*a_*DQ.i;
            h4 = cos(alpha_/2.0)+DQ.i*sin(alpha_/2.0);
            
            dq = h1*h2*h3*h4;
        end
        
        function dq_dot = dh2dq_dot(obj,q,ith)
            %   For a given link's Extended DH parameters, calculate the
            %   correspondent dual quaternion derivative of that joint's
            %   pose transformation with respect to its joint value.
            %   Usage: dq = dh2dq_dot(q,ith), where
            %          q: joint value
            %          ith: link number
            
            if nargin ~= 3
                error('Wrong number of arguments. The parameters are joint value and the correspondent link')
            end
            
            theta_ = obj.theta(ith);
            d_ = obj.d(ith);
            a_ = obj.a(ith);
            alpha_ = obj.alpha(ith);
            
            % If joint is rotational
            if obj.type(ith) == obj.JOINT_ROTATIONAL
                h1 = 0.5*( -sin( (theta_+q) /2.0) + DQ.k*cos( (theta_+q) /2.0) );
                h2 = 1 + DQ.E*0.5*d_*DQ.k;
                % If joint is prismatic
            else
                h1 = cos(theta_/2.0)+DQ.k*sin(theta_/2.0);
                h2 = DQ.E*0.5*DQ.k;
            end
            h3 = 1 + DQ.E*0.5*a_*DQ.i;
            h4 = cos(alpha_/2.0)+DQ.i*sin(alpha_/2.0);
            
            dq_dot = h1*h2*h3*h4;
        end
        
        function J = raw_pose_jacobian(obj,q,to_ith_link)
            % RAW_POSE_JACOBIAN(q) returns the Jacobian that satisfies
            % vec(x_dot) = J * q_dot, where x = fkm(q) and q is the
            % vector of joint variables.
            %
            % RAW_POSE_JACOBIAN(q,ith) returns the Jacobian that
            % satisfies vec(x_ith_dot) = J * q_dot(1:ith), where
            % x_ith = fkm(q, ith), that is, the fkm up to the i-th link.
            %
            % This function does not take into account any base or
            % end-effector displacements and should be used mostly
            % internally in DQ_kinematics
            
            if nargin < 3
                to_ith_link = obj.n_links;
            end
            
            J = zeros(8,to_ith_link);
            
            for i = 1:to_ith_link
                xi = DQ(1);
                for j = 1:to_ith_link
                    if i==j
                        xi = xi*obj.dh2dq_dot(q(j),j);
                    else
                        xi = xi*obj.dh2dq(q(j),j);
                    end
                end
                J(:,i) = vec8(xi);
            end
            
        end
        
    end
end