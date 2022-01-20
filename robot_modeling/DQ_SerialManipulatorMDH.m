% Concrete class that extends the DQ_SerialManipulator using the
% Denavit-Hartenberg parameters (DH)
%
% Usage: robot = DQ_SerialManipulatorDH(A)
% - 'A' is a 4 x n matrix containing the Denavit-Hartenberg parameters
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

% (C) Copyright 2022 DQ Robotics Developers
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
%     Juan Jose Quiroz Omana -  juanjqo@g.ecc.u-tokyo.ac.jp

classdef DQ_SerialManipulatorMDH < DQ_SerialManipulator
    
    properties (Access = protected)
        mdh_matrix_;
       
    end
    
    properties (Constant)
        % Joints that can be actuated
        % Rotational joint
        JOINT_ROTATIONAL = 1;
        % Prismatic joint
        JOINT_PRISMATIC = 2;
    end
    
    methods (Access = protected)
        function w = get_w(obj,ith) 
        % This method returns the term 'w' related with the time derivative of 
        % the unit dual quaternion pose using the Modified DH convention.
        % See. eq (2.27) of 'Two-arm Manipulation: From Manipulators to Enhanced 
        % Human-Robot Collaboration' by Bruno Adorno.
        % Usage: w = get_w(ith), where
        %          ith: link number
            joint_type = obj.get_types(ith); % obj.mdh_matrix_(5,ith);
            alpha = obj.get_alphas(ith);     % obj.mdh_matrix_(4,ith);
            a = obj.get_as(ith);             % obj.mdh_matrix_(3,ith);
            if joint_type == obj.JOINT_ROTATIONAL
                w = -DQ.j*sin(alpha)+ DQ.k*cos(alpha)...
                    -DQ.E*a*(DQ.j*cos(alpha) + DQ.k*sin(alpha));
            else               
                w = DQ.E*(cos(alpha)*DQ.k - sin(alpha)*DQ.j);
            end
        end
        
        function dq = mdh2dq(obj,q,ith)
            %   For a given link's Extended MDH parameters, calculate the correspondent dual
            %   quaternion
            %   Usage: dq = dh2dq(q,ith), where
            %          q: joint value
            %          ith: link number
            
            if nargin ~= 3
                error('Wrong number of arguments. The parameters are joint value and the correspondent link')
            end           

            % Store half angles and displacements
            
            half_theta = obj.get_thetas(ith)/2.0;  %obj.theta(ith)/2.0;
            d = obj.get_ds(ith); %obj.d(ith);
            a = obj.get_as(ith); %obj.a(ith);
            half_alpha = obj.get_alphas(ith)/2.0;  %obj.alpha(ith)/2.0;
            joint_type = obj.get_types(ith);
            
            % Add the effect of the joint value
            %if obj.type(ith) == obj.JOINT_ROTATIONAL
            if joint_type == obj.JOINT_ROTATIONAL   
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
            
            % Return the optimized standard dh2dq calculation
            
            
            dq = DQ([
                cosine_of_half_alpha*cosine_of_half_theta
                
                sine_of_half_alpha*cosine_of_half_theta
                
                -sine_of_half_alpha*sine_of_half_theta %MDH
                
                cosine_of_half_alpha*sine_of_half_theta
                
                -(a*sine_of_half_alpha*cosine_of_half_theta)  /2.0...
                - (d*cosine_of_half_alpha*sine_of_half_theta)/2.0
                
                (a*cosine_of_half_alpha*cosine_of_half_theta)/2.0...
                - (d*sine_of_half_alpha*sine_of_half_theta  )/2.0
                
                -(a*cosine_of_half_alpha*sine_of_half_theta)  /2.0... %MDH
                - (d*sine_of_half_alpha*cosine_of_half_theta)/2.0
                
                (d*cosine_of_half_alpha*cosine_of_half_theta)/2.0...
                - (a*sine_of_half_alpha*sine_of_half_theta  )/2.0
                ]);
        end        
    end
    
    methods
        function obj = DQ_SerialManipulatorMDH(A)
            % Constructor of the DQ_SerialManipulatorMDH class.           
            obj = obj@DQ_SerialManipulator(size(A,2));
            obj.mdh_matrix_ = A;            
            if nargin == 0
                error('Input: matrix whose columns contain the MDH parameters')
            end         
            
            if(size(A,1) ~= 5)
                error('Input: Invalid DH matrix. It should have 5 rows.')
            end
        end       
                                     
        
        function th = get_thetas(obj, ith)
            % GET_THETAS(ith) returns the ith element of the first row of the
            % Matrix mdh_matrix_, which correspond to the parameter 'theta' 
            % in the MDH convention.
            % If ith is not specified, GET_THETAS() returns the first row of the
            % Matrix mdh_matrix_.
            if nargin == 1
                th = obj.mdh_matrix_(1,:); %obj.theta;
            else
                obj.check_to_ith_link(ith);
                th = obj.mdh_matrix_(1,ith);
            end
        end
        
        function ds = get_ds(obj, ith)
            % GET_DS(ith) returns the ith element of the second row of the
            % Matrix mdh_matrix_, which correspond to the parameter 'd' 
            % in the MDH convention.
            % If ith is not specified, GET_DS() returns the second row of the
            % Matrix mdh_matrix_.
            if nargin == 1
                ds = obj.mdh_matrix_(2,:); %obj.d;
            else
                obj.check_to_ith_link(ith);
                ds = obj.mdh_matrix_(2,ith);
            end
        end
        
        function as = get_as(obj, ith)
            % GET_AS(ith) returns the ith element of the third row of the 
            % Matrix mdh_matrix_, which correspond to the parameter 'a' 
            % in the MDH convention.
            % If ith is not specified, GET_AS() returns the third row of the
            % Matrix mdh_matrix_.
            if nargin == 1
                as = obj.mdh_matrix_(3,:); %obj.a;
            else
                obj.check_to_ith_link(ith);
                as = obj.mdh_matrix_(3,ith); %obj.a;
            end
        end
        
        function alphas = get_alphas(obj, ith)
            % GET_ALPHAS(ith) returns the ith element of the fourth row of 
            % the Matrix mdh_matrix_, which correspond to the parameter 'alpha'
            % in the MDH convention.
            % If ith is not specified, GET_ALPHAS() returns the fourth row of the
            % Matrix mdh_matrix_.
            if nargin == 1
                alphas =  obj.mdh_matrix_(4,:); %obj.alpha; 
            else 
                obj.check_to_ith_link(ith);
                alphas =  obj.mdh_matrix_(4,ith);  
            end
        end
        
        function types = get_types(obj, ith)
            % GET_TYPES(ith) returns the ith element of the fifth row of the
            % Matrix mdh_matrix_, which correspond to the type of joints of the robot: 
            % DQ_SerialManipulatorDH.JOINT_ROTATIONAL
            % or DQ_SerialManipulatorDH.JOINT_PRISMATIC
            % If ith is not specified, GET_TYPES() returns the fifth row of the
            % Matrix mdh_matrix_.
            if nargin == 1
                types = obj.mdh_matrix_(5,:); %obj.type; 
            else
                obj.check_to_ith_link(ith);
                types = obj.mdh_matrix_(5,ith); 
            end
        end
        
        function J_dot = pose_jacobian_derivative(obj,q,q_dot, ith)
            % POSE_JACOBIAN_DERIVATIVE(q,q_dot) returns the Jacobian 
            % time derivative.
            % 
            % POSE_JACOBIAN_DERIVATIVE(q,q_dot,ith) returns the first
            % ith columns of the Jacobian time derivative.
            % This function does not take into account any base or
            % end-effector displacements.
            obj.check_q_vec(q);
            obj.check_q_vec(q_dot); 
            
            if nargin == 4
                n = ith;
                x_effector = obj.raw_fkm(q,ith);
                J = obj.raw_pose_jacobian(q,ith);
                vec_x_effector_dot = J*q_dot(1:ith);
            else
                n = obj.dim_configuration_space_;
                x_effector = obj.raw_fkm(q);
                J = obj.raw_pose_jacobian(q);
                vec_x_effector_dot = J*q_dot;
            end
            obj.check_to_ith_link(n);
                                 
            x = DQ(1);            
            J_dot = zeros(8,n);

            for i = 0:n-1
                % Use the standard DH convention                
                w = obj.get_w(i+1);               
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
                x = x*obj.mdh2dq(q(i+1),i+1);
            end
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
            
            obj.check_q_vec(q);
            
            if nargin == 3
                n = to_ith_link;
            else
                n = obj.dim_configuration_space_;
            end
            
            obj.check_to_ith_link(n);
            x = DQ(1);
            
            for i=1:n
                x = x*mdh2dq(obj,q(i),i);
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
            obj.check_q_vec(q);
            
            if nargin < 3
                to_ith_link = obj.dim_configuration_space_;
            end
            x_effector = obj.raw_fkm(q,to_ith_link);
            
            obj.check_to_ith_link(to_ith_link);
            x = DQ(1);
            J = zeros(8,to_ith_link);
            
            for i = 0:to_ith_link-1
                w = obj.get_w(i+1);
                z = 0.5*Ad(x,w);
                x = x*obj.mdh2dq(q(i+1),i+1);
                j = z * x_effector;
                J(:,i+1) = vec8(j);
            end
        end
        
    end
end