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
%       line_jacobian
%       plane_jacobian
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
        % Every robot receives a unique name, but users can choose any name
        % they want
        name;
    end
    
    properties (SetAccess = protected)
        % Reference frame used in fkm() and pose_jacobian methods
        reference_frame;
        % Frame used to determine the robot physical location
        base_frame;
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
        
        function ret = get_reference_frame(obj)
            ret = obj.reference_frame;
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
        
        function ret = get_base_frame(obj)
            ret = obj.base_frame;
        end
    end
    
    methods (Abstract)
        
        % GET_DIM_CONFIGURATION_SPACE returns the dimension of the configuration
        % space.
        dim = get_dim_configuration_space(obj);
        
        % FKM(q) takes the configuration vector 'q' and calculates the forward
        % kinematic model and returns the unit dual quaternion corresponding to 
        % the relevant pose. 
        % Optionally, in case of coupled kinematic chains, FKM(q, to_ith_link) 
        % calculates the forward kinematic model up to the ith element in
        % the chain.
        x = fkm(obj,q, to_ith_link);
        
        % POSE_JACOBIAN(q) returns the Jacobian that satisfies
        % vec8(x_pose_dot) = J * q_dot, where x_pose = fkm(q), 'x_pose_dot' is the time
        % derivative of 'x_pose' and 'q' is the configuration vector.
        % Optionally, in case of coupled kinematic chains, 
        % POSE_JACOBIAN(q, to_ith_link) calculates the forward kinematic model up to
        % the ith element in the chain.
        J = pose_jacobian(obj, q, to_ith_link);
    end
    
    methods(Static)
        
        function Jd = distance_jacobian(J_pose, x_pose)
         % Given the Jacobian 'J_pose' and the corresponding unit dual quaternion 'x_pose' 
         % that satisfy vec8(x_pose_dot) = J_pose * q_dot, DISTANCE_JACOBIAN(J_pose,x_pose) returns 
         % the distance Jacobian; that it, the Jacobian that satisfies the 
         % relation dot(d^2) = Jd * q_dot, where dot(d^2) is the time 
         % derivative of the square of the distance between the origin of the 
         % frame represented by 'x_pose' and the origin of the reference frame.
             if ~is_unit(x_pose)
                error(['The second argument of distance_jacobian should be'...
                        ' a unit dual quaternion']);
             end
             p = translation(x_pose);
             Jp = DQ_Kinematics.translation_jacobian(J_pose,x_pose);
             Jd = 2*vec4(p)'*Jp;
        end
        
        function Jlx = line_jacobian(pose_jacobian, x, line_direction)
        % LINE_JACOBIAN(pose_jacobian, x, line_direction) returns the line
        % Jacobian related to a line, whose direction with respect to the 
        % end-effector frame, given by the unit dual quaternion 'x', is given by
        % 'line_direction' and passes through the origin of 'x'. 
        %
        % For example, using i_, j_, and k_ will return the line Jacobian related
        % to the line collinear with, respectively, the x-axis, y-axis, and 
        % z-axis of 'x'.
        %
        % For more details see Eq. 20 of?Marinho, M. M., Adorno, B. V., 
        % Harada, K., & Mitsuishi, M. "Active Constraints Using Vector
        % Field Inequalities for Surgical Robots." ICRA 2018.
        % https://doi.org/10.1109/ICRA.2018.8461105

            Jt = DQ_Kinematics.translation_jacobian(pose_jacobian,x);
            Jr = DQ_Kinematics.rotation_jacobian(pose_jacobian);

            t = translation(x);
            r = rotation(x);

            l = r*(line_direction)*r';

            % Line direction and moment Jacobians
            Jrx = (haminus4(line_direction*r') + ...
                               hamiplus4(r*line_direction)*DQ.C4)*Jr;
            Jmx = crossmatrix4(l)'*Jt + crossmatrix4(t)*Jrx;

            % Assemble the 8-by-n line Jacobian, where n = dim_configuration_space          
            Jlx = [Jrx; Jmx];
        end
        
        function Jr = rotation_jacobian(J_pose)
        % Given the Jacobian 'J_pose' and the corresponding unit dual quaternion 'x_pose' 
        % that satisfy vec8(x_pose_dot) = J_pose * q_dot, ROTATION_JACOBIAN(J_pose) returns 
        % the Jacobian Jr that satisfies vec4(r_dot) = Jr * q_dot, where r_dot 
        % is the time derivative of the rotation quaternion r in 
        % x_pose = r + DQ.E*(1/2)*p*r and q_dot is the time derivative of the 
        % configuration vector.
            Jr = J_pose(1:4,:);
        end
        
        function Jplane = plane_jacobian(pose_jacobian, x_pose, plane_normal)
        % PLANE_JACOBIAN(pose_jacobian, x_pose, plane_normal) returns the plane
        % jacobian, where x_pose
        % is the end-effector pose, pose_jacobian is the matrix that
        % satisfies x_pose_dot = pose_jacobian * q_dot, with q_dot being
        % the joint velocities, and plane_normal is the plane normal with
        % respect to the reference frame. For example using i_, j_, and k_
        % will return the plane Jacobian whose normal is collinear with, 
        % respectively, the x-axis, y-axis, and z-axis of the end-effector
        % frame given by x_pose
        %
        % For more details, see Section IV.F?of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270
       
            % Requirements
            xt = translation(x_pose);
            xr = rotation(x_pose);
            Jr = DQ_Kinematics.rotation_jacobian(pose_jacobian);
            Jt = DQ_Kinematics.translation_jacobian(pose_jacobian, x_pose);

            % Plane normal w.r.t the reference frame
            nz = xr*(plane_normal)*xr';

            % Plane normal Jacobian, that is, the time derivative of the plane
            % normal w.r.t the reference frame
            Jnz = (haminus4(plane_normal*xr') + ...
                hamiplus4(xr*plane_normal)*DQ.C4)*Jr;

            % Plane distance Jacobian, that is, the time derivative of the
            % plane distance w.r.t the reference frame
            Jdz  = vec4(nz)'*Jt + vec4(xt)'*Jnz;

            % Plane Jacobian that relates the joint velocities with the time 
            % derivative of the plane given by n + E_ * d, where n is the
            % plane normal and d is the distance with respect to the
            % reference frame.
            Jplane = [Jnz;Jdz;zeros(3,size(pose_jacobian,2))];
        end

        
        function Jp = translation_jacobian(J_pose,x_pose)
        % Given the Jacobian 'J_pose' and the corresponding unit dual quaternion 'x_pose' 
        % that satisfy vec8(x_pose_dot) = J_pose * q_dot, TRANSLATION_JACOBIAN(J_pose,x_pose) 
        % returns the Jacobian that satisfies the relation 
        % vec4(p_dot) = Jp * q_dot, where p_dot is the time derivative of the
        % translation quaternion p and q_dot is the time derivative of the 
        % configuration vector
            if ~is_unit(x_pose)
                error(['The second argument of translation_jacobian should be'...
                    ' a unit dual quaternion']);              
            end
            Jp = 2*haminus4(x_pose.P')*J_pose(5:8,:)+2*hamiplus4(x_pose.D)*DQ.C4*J_pose(1:4,:);
        end
        
      
        
       
 
 
        
    end
end