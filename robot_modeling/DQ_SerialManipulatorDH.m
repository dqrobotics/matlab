% Concrete class that extends the DQ_SerialManipulator using the
% Denavit-Hartenberg parameters (DH)
%
% Usage: robot = DQ_SerialManipulatorDH(A)
% - 'A' is a 5 x n matrix containing the Denavit-Hartenberg parameters
%   (n is the number of links)
%    A = [theta1 ... thetan;
%            d1  ...   dn;
%            a1  ...   an;
%         alpha1 ... alphan;
%         type1  ... typen]
% where type is the actuation type, either DQ_SerialManipulatorDH.JOINT_ROTATIONAL
% or DQ_SerialManipulatorDH.JOINT_PRISMATIC
% - The only accepted convention in this subclass is the 'standard' DH
% convention.
%
% If the joint is of type JOINT_ROTATIONAL, then the first row of A will
% have the joint offsets. If the joint is of type JOINT_PRISMATIC, then the
% second row of A will have the joints offsets.
%
% DQ_SerialManipulatorDH Methods (Concrete):
%       get_dim_configuration_space - Return the dimension of the configuration space.
%       fkm - Compute the forward kinematics while taking into account base and end-effector's rigid transformations.
%       plot - Plots the serial manipulator.
%       pose_jacobian - Compute the pose Jacobian while taking into account base's and end-effector's rigid transformations.
%       pose_jacobian_derivative - Compute the time derivative of the pose Jacobian.
%       raw_fkm - Compute the FKM without taking into account base's and end-effector's rigid transformations.
%       raw_pose_jacobian - Compute the pose Jacobian without taking into account base's and end-effector's rigid transformations.
%       set_effector - Set an arbitrary end-effector rigid transformation with respect to the last frame in the kinematic chain.
% See also DQ_SerialManipulator.

% (C) Copyright 2020-2022 DQ Robotics Developers
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
%     1. Bruno Vihena Adorno (adorno@ieee.org)
%        Responsible for the original implementation in file SerialManipulator.m 
%        [bvadorno committed on Apr 10, 2019] (bc7a95f)
%        (https://github.com/dqrobotics/matlab/blob/bc7a95f064b15046f43421d418946f60b1b33058/robot_modeling/DQ_SerialManipulator.m).
%
%     2. Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
%        - Created this file by reorganizing the code in the original file to comply 
%          with SerialManipulator.m becoming an abstract class, according to the discussion 
%          at #56 (https://github.com/dqrobotics/matlab/pull/56).
% 
%        - Added support for prismatic joints. 
%          [mmmarinho committed on Apr 28, 2020] (f5aa70a) 
%          https://github.com/dqrobotics/matlab/commit/f5aa70ac6a0a676557543e2bf7c418ab05c47326
%
%     3. Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
%        - Added some modifications discussed at #56 (https://github.com/dqrobotics/matlab/pull/75)
%          to define DQ_SerialManipulator as an abstract class.           

classdef DQ_SerialManipulatorDH < DQ_SerialManipulator
    properties
        theta,d,a,alpha;
        type
    end
    
    properties (Constant)
        % Joints that can be actuated
        % Rotational joint
        JOINT_ROTATIONAL = 1;
        % Prismatic joint
        JOINT_PRISMATIC = 2;
    end

    methods (Access = protected)
        function dq = dh2dq(obj,q,ith)
            %   DQ2DQ(q, ith) calculates  the corresponding dual quaternion for
            %   a given link's DH parameters
            %
            %   Usage: dq = dh2dq(q,ith), where
            %          q: joint value
            %          ith: link number
            %
            %   Eq. (2.34) of Adorno, B. V. (2011). Two-arm Manipulation: From Manipulators
            %   to Enhanced Human-Robot Collaboration [Contribution à la manipulation à deux bras : 
            %   des manipulateurs à la collaboration homme-robot]. 
            %   https://tel.archives-ouvertes.fr/tel-00641678/
            
            if nargin ~= 3
                error('Wrong number of arguments. The parameters are joint value and the correspondent link')
            end
            
            % Store half angles and displacements
            half_theta = obj.theta(ith)/2.0;
            d = obj.d(ith);
            a = obj.a(ith);
            half_alpha = obj.alpha(ith)/2.0;
            
            % Add the effect of the joint value
            if obj.type(ith) == obj.JOINT_ROTATIONAL
                % If joint is rotational
                half_theta = half_theta + (q/2.0);
            else
                % If joint is prismatic
                d = d + q;
            end
            
            % Pre-calculate cosines and sines
            sine_of_half_theta = sin(half_theta);
            cosine_of_half_theta = cos(half_theta);
            sine_of_half_alpha = sin(half_alpha);
            cosine_of_half_alpha = cos(half_alpha);
            
            % Return the standard dh2dq calculation
            d2 = d/2;
            a2 = a/2;
            h(1) = cosine_of_half_alpha*cosine_of_half_theta;
            h(2) = sine_of_half_alpha*cosine_of_half_theta;
            h(3) = sine_of_half_alpha*sine_of_half_theta;
            h(4) = cosine_of_half_alpha*sine_of_half_theta;
            h(5) = -a2*h(2) - d2*h(4);
            h(6) =  a2*h(1) - d2*h(3);
            h(7) =  a2*h(4) + d2*h(2);
            h(8) = d2*h(1)  - a2*h(3);
            dq = DQ(h);
        end
        
        function w = get_w(obj,ith)  
        % This method returns the term 'w' related with the time derivative of 
        % the unit dual quaternion pose using the Standard DH convention.
        % See. eq (2.32) of 'Two-arm Manipulation: From Manipulators to Enhanced 
        % Human-Robot Collaboration' by Bruno Adorno.
        % Usage: w = get_w(ith), where
        %          ith: link number    
            if obj.type(ith) == obj.JOINT_ROTATIONAL
                w = DQ.k;
            else
                % see Table 1 of "Dynamics of Mobile Manipulators using Dual Quaternion Algebra."
                % by Silva, F. F. A., Quiroz-Omaña, J. J., and Adorno, B. V. (April 12, 2022).  
                % ASME. J. Mechanisms Robotics. doi: https://doi.org/10.1115/1.4054320
                w = DQ.E*DQ.k;
            end
        end
    end
    
    methods
        function obj = DQ_SerialManipulatorDH(A, convention)
            % These are initialized in the constructor of
            % DQ_SerialManipulator 
            % obj.dim_configuration_space = dim_configuration_space;

            str = ['DQ_SerialManipulatorDH(A), where ' ...
                   'A = [theta1 ... thetan; ' ...
                   ' d1  ...   dn; ' ...
                   ' a1  ...   an; ' ...
                   ' alpha1 ... alphan; ' ...
                   ' type1  ... typen]'];
            
            
            if nargin == 0
                error(['Input: matrix whose columns contain the DH parameters' ...
                       ' and type of joints. Example: ' str])
            end

            if nargin == 2
                warning(['DQ_SerialManipulatorDH(A, convention) is deprecated.' ...
                        ' Please use DQ_SerialManipulatorDH(A) instead.']);    
            end
            
            if(size(A,1) ~= 5)
                error('Input: Invalid DH matrix. It must have 5 rows.')
            end

            % n_links 
            % TODO: change n_links to dim_configuration_space
            obj.n_links = size(A,2);

            % Add theta, d, a, alpha and type
            obj.theta = A(1,:);
            obj.d     = A(2,:);
            obj.a     = A(3,:);
            obj.alpha = A(4,:);
            obj.type  = A(5,:);
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
            x_effector = obj.raw_fkm(q,to_ith_link);
            
            x = DQ(1);
            J = zeros(8,to_ith_link);
            
            for i = 0:to_ith_link-1
                w = obj.get_w(i+1);
                z = 0.5*Ad(x,w);
                x = x*obj.dh2dq(q(i+1),i+1);
                j = z * x_effector;
                J(:,i+1) = vec8(j);
            end
        end

        function J_dot = raw_pose_jacobian_derivative(obj,q,q_dot, to_ith_link)
            % RAW_POSE_JACOBIAN_DERIVATIVE(q,q_dot) returns the Jacobian 
            % time derivative.
            % 
            % RAW_POSE_JACOBIAN_DERIVATIVE(q,q_dot,to_ith_link) returns the first
            % to_ith_link columns of the Jacobian time derivative.
            % This function does not take into account any base or
            % end-effector displacements.
            % obj.check_q_vec(q);
            % obj.check_q_vec(q_dot);            
            
            if nargin == 4
                n = to_ith_link;
                x_effector = obj.raw_fkm(q,to_ith_link);
                J = obj.raw_pose_jacobian(q,to_ith_link);
                vec_x_effector_dot = J*q_dot(1:to_ith_link);
            else
                n = obj.n_links;
                % obj.check_to_ith_link(n);
                x_effector = obj.raw_fkm(q);
                J = obj.raw_pose_jacobian(q);
                vec_x_effector_dot = J*q_dot;
            end
                                
            x = DQ(1);            
            J_dot = zeros(8,n);

            for i = 0:n-1
                % Use the standard DH convention
                
                w = obj.get_w(i+1); %w = DQ.k;
                %z = DQ(obj.get_z(x.q));
                z = 0.5*x*w*conj(x);
                
                % When i = 0 and length(theta) = 1, theta(1,i) returns
                % a 1 x 0 vector, differently from the expected
                % behavior, which is to return a 0 x 1 matrix.
                % Therefore, we have to deal with the case i = 0
                % explictly.
                if i ~= 0
                    vec_zdot = 0.5*(haminus8(w*x') + ...
                        hamiplus8(x*w)*DQ.C8) * ...
                        obj.raw_pose_jacobian(q,i)*q_dot(1:i);
                else
                    vec_zdot = zeros(8,1);
                end
                J_dot(:,i+1) = haminus8(x_effector)*vec_zdot +...
                    hamiplus8(z)*vec_x_effector_dot;
                x = x*obj.dh2dq(q(i+1),i+1);
            end
        end
        
    end
end