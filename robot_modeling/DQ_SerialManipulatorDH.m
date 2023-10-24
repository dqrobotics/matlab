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
% where type is the actuation type, either DQ_JointType.REVOLUTE
% or DQ_JointType.PRISMATIC
% - The only accepted convention in this subclass is the 'standard' DH
% convention.
%
% If the joint is of type REVOLUTE, then the first row of A will
% have the joint offsets. If the joint is of type PRISMATIC, then the
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
%       raw_pose_jacobian_derivative - Compute the pose Jacobian derivative without taking into account base's and end-effector's rigid transformations.
%       set_effector - Set an arbitrary end-effector rigid transformation with respect to the last frame in the kinematic chain.
%       get_parameters - Return a vector containing the DH parameters.
%       get_parameter -  Return the DH parameter of the ith joint.
%       set_parameters - Set the DH parameters.
%       set_parameter -  Set the DH parameter of the ith joint.
% See also DQ_SerialManipulator.

% (C) Copyright 2020-2023 DQ Robotics Developers
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
%        - Added some modifications discussed at #75 (https://github.com/dqrobotics/matlab/pull/75)
%          to define DQ_SerialManipulator as an abstract class.   
%
%        - Added the following methods: get_parameter, get_parameters,
%          set_parameter, and set_parameters.

classdef DQ_SerialManipulatorDH < DQ_SerialManipulator
    properties (Access = protected)
        theta,d,a,alpha;
    end
    
    properties (Constant)
        % Joints that can be actuated
        % Rotational joint
        JOINT_ROTATIONAL = 1; % Deprecated
        % Prismatic joint
        JOINT_PRISMATIC = 2;  % Deprecated
    end

    methods (Access = protected)
        function dq = get_link2dq(obj,q,ith)
            %   GET_LINK2DQ(q, ith) calculates  the corresponding dual quaternion for
            %   a given link's DH parameters
            %
            %   Usage: dq = get_link2dq(q,ith), where
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
            if obj.get_joint_type(ith) == DQ_JointType.REVOLUTE
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
            if obj.get_joint_type(ith) == DQ_JointType.REVOLUTE
                w = DQ.k;
            else
                % see Table 1 of "Dynamics of Mobile Manipulators using Dual Quaternion Algebra."
                % by Silva, F. F. A., Quiroz-Omaña, J. J., and Adorno, B. V. (April 12, 2022).  
                % ASME. J. Mechanisms Robotics. doi: https://doi.org/10.1115/1.4054320
                w = DQ.E*DQ.k;
            end
        end
    end

    methods (Static, Access = protected) 
         function ret = get_supported_joint_types()
         % This method returns the supported joint types.
            ret = [DQ_JointType.REVOLUTE, DQ_JointType.PRISMATIC];
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
            
            obj.dim_configuration_space = size(A,2);

            % Add theta, d, a, alpha and type
            obj.theta = A(1,:);
            obj.d     = A(2,:);
            obj.a     = A(3,:);
            obj.alpha = A(4,:);
            obj.set_joint_types(A(5,:));
        end

        function ret = get_parameters(obj, parameterType)
            % This method returns a vector containing the DH parameters.
            % Usage: get_parameters(parameterType)
            %           parameterType: Parameter type, which corresponds to
            %                          "THETA", "D", "A", or "ALPHA".
            arguments
               obj
               parameterType DQ_ParameterDH              
            end

            switch parameterType
                case DQ_ParameterDH.THETA
                    ret = obj.theta;
                case DQ_ParameterDH.D
                    ret = obj.d;
                case DQ_ParameterDH.A
                    ret = obj.a;
                case DQ_ParameterDH.ALPHA
                    ret = obj.alpha;                    
            end            
        end

        function ret = get_parameter(obj, parameterType, ith_joint)
            % This method returns the DH parameter of the ith joint.
            % Usage: get_parameter(parameterType, ith_joint)
            %           parameterType: Parameter type, which corresponds to
            %                          "THETA", "D", "A", or "ALPHA".
            %           ith_joint: Joint number.
            arguments
               obj
               parameterType DQ_ParameterDH 
               ith_joint int32
            end

            switch parameterType
                case DQ_ParameterDH.THETA
                    ret = obj.theta(ith_joint);
                case DQ_ParameterDH.D
                    ret = obj.d(ith_joint);
                case DQ_ParameterDH.A
                    ret = obj.a(ith_joint);
                case DQ_ParameterDH.ALPHA
                    ret = obj.alpha(ith_joint);                    
            end 
        end

        function set_parameters(obj, parameterType, vector_parameters)
            % This method sets the DH parameters.
            % Usage: set_parameters(parameterType, vector_parameters)
            %           parameterType: Parameter type, which corresponds to
            %                          "THETA", "D", "A", or "ALPHA".
            %           vector_parameters: Vector containing the new
            %                              parameters.
            arguments
               obj
               parameterType DQ_ParameterDH   
               vector_parameters double
            end

            obj.check_q_vec(vector_parameters);
            vector_parameters = reshape(vector_parameters, [1, obj.get_dim_configuration_space()]);

            switch parameterType
                case DQ_ParameterDH.THETA
                    obj.theta    =  vector_parameters;
                case DQ_ParameterDH.D
                    obj.d        =  vector_parameters;
                case DQ_ParameterDH.A
                    obj.a        =  vector_parameters;
                case DQ_ParameterDH.ALPHA
                    obj.alpha    =  vector_parameters;                  
            end 
        end

        function set_parameter(obj, parameterType, ith_joint, parameter)
            % This method sets the DH parameter of the ith joint.
            % Usage: set_parameters(parameterType, vector_parameters)
            %           parameterType: Parameter type, which corresponds to
            %                          "THETA", "D", "A", or "ALPHA".
            %           ith_joint: Joint number.
            %           parameter: The new parameter.
            arguments
               obj
               parameterType DQ_ParameterDH 
               ith_joint int32
               parameter double
            end

            obj.check_ith_link(ith_joint);

            switch parameterType
                case DQ_ParameterDH.THETA
                    obj.theta(ith_joint) =  parameter;
                case DQ_ParameterDH.D
                    obj.d(ith_joint)     =  parameter;
                case DQ_ParameterDH.A
                    obj.a(ith_joint)     =  parameter;
                case DQ_ParameterDH.ALPHA
                    obj.alpha(ith_joint) =  parameter;                  
            end 
        end
    
        
    end
end