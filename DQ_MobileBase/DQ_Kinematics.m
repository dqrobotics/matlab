% CLASS DQ_Kinematics
% 
% Abstract class that defines an interface for all subclasses that need to
% implement robot kinematics.
%
% For more information about the available methods, see also
% Abstract:
%       get_dim_configuration_space
%       fkm
%       pose_jacobian
% Concrete:
%       set_base_frame
%       set_reference_frame
% Static:
%       distance_jacobian
%       rotation_jacobian
%       translation_jacobian

% (C) Copyright 2011-2019 DQ Robotics Developers
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
%     Bruno Vihena Adorno - adorno@ufmg.br

classdef DQ_Kinematics < handle
    % DQ_Kinematics inherits the HANDLE superclass to avoid unnecessary copies
    % when passing DQ_Kinematics objects as arguments to methods.
    properties
        % Reference frame used in fkm() and pose_jacobian methods
        reference_frame; 
        % Frame used to determine the robot physical location
        base_frame;
        % Every robot receives a unique name, but users can choose any name
        % they want
        name;
    end
    
    properties (SetAccess = protected)
        % Robot configuration vector
        q        
    end
    
    methods
        function obj = DQ_Kinematics()
            % Both reference_frame and base_frame initially coincide, but
            % this can be changed by using set_reference_frame() and 
            % set_base_frame()
            obj.reference_frame = DQ(1); 
            obj.base_frame = DQ(1);            
            % Define a unique robot name
            obj.name = sprintf('%f',rand(1));
        end
        
        function set_reference_frame(obj,reference_frame)
            % sets the reference frame used for the fkm() and pose_jacobian() 
            % methods   
            if is_unit(reference_frame)
                obj.reference_frame = reference_frame;
            else
                error('The reference frame must be a unit dual quaternion.');
            end
        end
        
        function set_base_frame(obj, base_frame)
            % set_base_frame(base_frame) sets the base frame with respect
            % to the global reference frame (i.e., the identity). The rigid 
            % motion from the global reference frame to the robot base is given
            % by the unit dual quaternion 'base_frame'. This function is used to
            % define the 'physical' place of the robot base
            % and it does not necessarily coincides with the reference
            % frame.
            if is_unit(base_frame)
                obj.base_frame = base_frame;
            else
                error('The base frame must be a unit dual quaternion.');
            end
        end
    end
    
    methods (Abstract)
        
        % GET_DIM_CONFIGURATION_SPACE returns the dimension of the configuration
        % space.
        dim = get_dim_configuration_space(obj);
        
        % FKM(q) takes the configuration vector 'q' and calculates the forward
        % kinematic model and returns the unit dual quaternion corresponding to 
        % the relevant pose. 
        % Optionally, in case of coupled kinematic chains, FKM(theta, ith) 
        % calculates the forward kinematic model up to the ith element in
        % the chain.
        x = fkm(obj,theta, ith);
        
        % POSE_JACOBIAN(q) returns the Jacobian that satisfies
        % vec8(x_dot) = J * q_dot, where x = fkm(q), 'x_dot' is the time
        % derivative of 'x' and 'q' is the configuration vector.
        % Optionally, in case of coupled kinematic chains, 
        % POSE_JACOBIAN(theta, ith) calculates the forward kinematic model up to
        % the ith element in the chain.
        J = pose_jacobian(obj, theta, ith);
    end
    
    methods(Static)
        function Jd = distance_jacobian(J,x)
         % Given the Jacobian 'J' and the corresponding unit dual quaternion 'x' 
         % that satisfy vec8(x_dot) = J * q_dot, DISTANCE_JACOBIAN(J,x) returns 
         % the distance Jacobian; that it, the Jacobian that satisfies the 
         % relation dot(d^2) = Jd * q_dot, where dot(d^2) is the time 
         % derivative of the square of the distance between the origin of the 
         % frame represented by 'x' and the origin of the reference frame.
             if ~is_unit(x)
                error(['The second argument of distance_jacobian should be'...
                        ' a unit dual quaternion']);
             end
             p = translation(x);
             Jp = DQ_Kinematics.translation_jacobian(J,x);
             Jd = 2*vec4(p)'*Jp;
        end
    
        
        function Jp = translation_jacobian(J,x)
        % Given the Jacobian 'J' and the corresponding unit dual quaternion 'x' 
        % that satisfy vec8(x_dot) = J * q_dot, TRANSLATION_JACOBIAN(J,x) 
        % returns the Jacobian that satisfies the relation 
        % vec4(p_dot) = Jp * q_dot, where p_dot is the time derivative of the
        % translation quaternion p and q_dot is the time derivative of the 
        % configuration vector
            if ~is_unit(x)
                error(['The second argument of translation_jacobian should be'...
                    ' a unit dual quaternion']);              
            end
            Jp = 2*haminus4(x.P')*J(5:8,:)+2*hamiplus4(x.D)*DQ.C4*J(1:4,:);
        end
        
        function Jr = rotation_jacobian(J)
        % Given the Jacobian 'J' and the corresponding unit dual quaternion 'x' 
        % that satisfy vec8(x_dot) = J * q_dot, ROTATION_JACOBIAN(J) returns 
        % the Jacobian Jr that satisfies vec4(r_dot) = Jr * q_dot, where r_dot 
        % is the time derivative of the rotation quaternion r in 
        % x = r + DQ.E*(1/2)*p*r and q_dot is the time derivative of the 
        % configuration vector.
            Jr = J(1:4,:);
        end
    end
end