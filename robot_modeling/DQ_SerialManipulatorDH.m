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
% where type is the actuation type, either DQ_SerialManipulatorDH.REVOLUTE
% or DQ_SerialManipulatorDH.PRISMATIC
% - The only accepted convention in this subclass is the 'standard' DH
% convention.
%
% If the joint is of type REVOLUTE, then the first row of A will
% have the joint offsets. If the joint is of type PRISMATIC, then the
% second row of A will have the joints offsets.
%
% DQ_SerialManipulatorDH Methods (Concrete):
%       get_dh_parameters_theta - Returns the vector containing the theta parameters of the DH table. 
%       get_dh_parameters_d - Returns the vector containing the d parameters of the DH table.
%       get_dh_parameters_a - Returns the vector containing the a parameters of the DH table.
%       get_dh_parameters_alpha - Returns the vector containing the alpha parameters of the DH table.
%       get_joint_types - Returns the joint type, which can be either REVOLUTE or PRISMATIC.
%       pose_jacobian_derivative - Compute the time derivative of the pose Jacobian.
%       raw_pose_jacobian - Compute the pose Jacobian without taking into account base's and end-effector's rigid transformations.
%       raw_fkm - Compute the FKM without taking into account base's and end-effector's rigid transformations.
%
%
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
%     Bruno Vihena Adorno - bruno.adorno@manchester.ac.uk
%     Murilo M. Marinho - murilo@nml.t.u-tokyo.ac.jp
%     Juan Jose Quiroz Omana - juanjqo@g.ecc.u-tokyo.ac.jp

classdef DQ_SerialManipulatorDH < DQ_SerialManipulator
  
    
    properties (Access = protected)
        dh_matrix_;
       
    end
    
    properties (Constant)
        % Joints that can be actuated
        % Revolute joint
        JOINT_ROTATIONAL = 1;
        % Prismatic joint
        JOINT_PRISMATIC = 2;
    end
    
    methods (Access = protected)       
        function w = get_w(obj,ith) 
        % This method returns the term 'w' related with the time derivative of 
        % the unit dual quaternion pose using the Standard DH convention.
        % See. eq (2.32) of 'Two-arm Manipulation: From Manipulators to Enhanced 
        % Human-Robot Collaboration' by Bruno Adorno.
        % Usage: w = get_w(ith), where
        %          ith: link number
            joint_type = obj.dh_matrix_(5,ith);
            if joint_type == DQ_JointType.REVOLUTE
                w = DQ.k;
            else
                % see Table 1 of "Dynamics of Mobile Manipulators using Dual Quaternion Algebra."
                % by Silva, F. F. A., Quiroz-Omaña, J. J., and Adorno, B. V. (April 12, 2022).  
                % ASME. J. Mechanisms Robotics. doi: https://doi.org/10.1115/1.4054320
                w = DQ.E*DQ.k;
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
            
            %The unoptimized standard dh2dq calculation is commented below
            %if obj.type(ith) == obj.JOINT_ROTATIONAL
            %    % If joint is rotational
            %    h1 = cos((obj.theta(ith)+q)/2.0)+DQ.k*sin((obj.theta(ith)+q)/2.0);
            %    h2 = 1 + DQ.E*0.5*obj.d(ith)*DQ.k;
            %else
            %    % If joint is prismatic
            %    h1 = cos(obj.theta(ith)/2.0)+DQ.k*sin(obj.theta(ith)/2.0);
            %    h2 = 1 + DQ.E*0.5*(obj.d(ith)+q)*DQ.k;
            %end
            %h3 = 1 + DQ.E*0.5*obj.a(ith)*DQ.i;
            %h4 = cos(obj.alpha(ith)/2.0)+DQ.i*sin(obj.alpha(ith)/2.0);
            %dq = h1*h2*h3*h4;
            
            % The optimized standard dh2dq calculation
            % Store half angles and displacements
            half_theta = obj.dh_matrix_(1,ith)/2.0; %obj.theta(ith)/2.0; 
            d = obj.dh_matrix_(2,ith); %obj.d(ith);
            a = obj.dh_matrix_(3,ith); %obj.a(ith);
            half_alpha = obj.dh_matrix_(4,ith)/2.0; %obj.alpha(ith)/2.0;
            joint_type = obj.dh_matrix_(5,ith);
            
            % Add the effect of the joint value            
            if joint_type == DQ_JointType.REVOLUTE 
                % If joint is revolute
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

%             dq = DQ([
%                 cosine_of_half_alpha*cosine_of_half_theta
%                 
%                 sine_of_half_alpha*cosine_of_half_theta
%                 
%                 sine_of_half_alpha*sine_of_half_theta
%                 
%                 cosine_of_half_alpha*sine_of_half_theta
%                 
%                 -(a*sine_of_half_alpha*cosine_of_half_theta)  /2.0...
%                 - (d*cosine_of_half_alpha*sine_of_half_theta)/2.0
%                 
%                 (a*cosine_of_half_alpha*cosine_of_half_theta)/2.0...
%                 - (d*sine_of_half_alpha*sine_of_half_theta  )/2.0
%                 
%                 (a*cosine_of_half_alpha*sine_of_half_theta)  /2.0...
%                 + (d*sine_of_half_alpha*cosine_of_half_theta)/2.0
%                 
%                 (d*cosine_of_half_alpha*cosine_of_half_theta)/2.0...
%                 - (a*sine_of_half_alpha*sine_of_half_theta  )/2.0
%                 ]);

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
        
    end
    
    methods
        function obj = DQ_SerialManipulatorDH(A,convention)
            % These are initialized in the constructor of
            % DQ_SerialManipulator, where A is given as follows
            %    A = [theta1 ... thetan;
            %            d1  ...   dn;
            %            a1  ...   an;
            %         alpha1 ... alphan;
            %         type1  ... typen]
            
              
            obj = obj@DQ_SerialManipulator(size(A,2));
            obj.dh_matrix_ = A;
           
            if nargin == 0
                error('Input: matrix whose columns contain the DH parameters')
            end            
            if nargin == 2
                warning('DQ_SerialManipulatorDH(A,convention) is deprecated. Please use DQ_SerialManipulatorDH(A) instead.');
                
            end
            
            if(size(A,1) ~= 5)
                error('Input: Invalid DH matrix. It should have 5 rows.')
            end
           
        end      
                      
       
        
        function th = get_dh_parameters_theta(obj)
            %GET_DH_PARAMETERS_THETA() Returns the vector containing the theta parameters of the DH table.
            th = obj.dh_matrix_(1,:); %obj.theta;
        end
        
        function ds = get_dh_parameters_d(obj)
            % GET_DH_PARAMETERS_D() Returns the vector containing the d parameters of the DH table.
            ds = obj.dh_matrix_(2,:); %obj.d;
        end
        
        function as = get_dh_parameters_a(obj)
            % GET_DH_PARAMETERS_A() Returns the vector containing the a parameters of the DH table.
            as = obj.dh_matrix_(3,:); %obj.a;
        end
        
        function alphas = get_dh_parameters_alpha(obj)
            % GET_DH_PARAMETERS_ALPHA() Returns the vector containing the alpha parameters of the DH table.
            alphas =  obj.dh_matrix_(4,:); %obj.alpha;            
        end
        
        function types = get_joint_types(obj)
            % GET_JOINT_TYPES() Returns the joint type, which can be either REVOLUTE or PRISMATIC.
            types = obj.dh_matrix_(5,:); %obj.type; 
        end
        
        % TODO:
        % This method is not defined in the DQ_Kinematics superclass
        % we need to fix it in the future.
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
                obj.check_to_ith_link(n);
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
            obj.check_q_vec(q);

            if nargin == 3
                n = to_ith_link;
            else
                n = obj.dim_configuration_space_;
            end

            obj.check_to_ith_link(n);
            x_effector = obj.raw_fkm(q,n);
          
            x = DQ(1);
            J = zeros(8,n);
            
            for i = 1:n
                w = obj.get_w(i);
                z = 0.5*Ad(x,w);
                x = x*obj.dh2dq(q(i),i);
                j = z * x_effector;
                J(:,i) = vec8(j);
            end
        end
        
    end
end

