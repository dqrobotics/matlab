% Abstract class that defines an interface to implement robot kinematics.
%
% DQ_Kinematics Properties:
%   base_frame - Frame used to determine the robot physical location.
%   name - Unique name that is generated randomly.
%   reference_frame - Reference frame (not always coincident with base_frame).
%   q - Robot configuration vector.
%
% DQ_Kinematics Methods (Abstract):
%       get_dim_configuration_space - Returns the dimension of the configuration space.
%       fkm - Compute the forward kinematics.
%       pose_jacobian - Compute the Jacobian that maps the configurations velocities to the time derivative of the pose of a frame attached to the robot.
% DQ_Kinematics Methods (Concrete):
%       set_base_frame - Set the physical location of the robot in space.
%       set_reference_frame - Set the reference frame for all calculations.
% DQ_Kinematics Methods (Static):
%       distance_jacobian - Compute the (squared) distance Jacobian.
%       line_jacobian - Compute the line Jacobian.
%       line_to_line_distance_jacobian  - Compute the line-to-line distance Jacobian.
%       line_to_line_residual - Compute the line-to-line residual.
%       line_to_point_distance_jacobian - Compute the line-to-line distance Jacobian.
%       line_to_point_residual - Compute the line-to-point residual.
%       plane_jacobian - Compute the plane Jacobian.
%       plane_to_point_distance_jacobian - Compute the plane-to-point distance Jacobian.
%       plane_to_point_residual - Compute the plane-to-point residual.
%       point_to_line_distance_jacobian - Compute the point-to-line distance Jacobian.
%       point_to_line_residual - Compute the point to line residual.
%       point_to_plane_distance_jacobian - Compute the point to plane distance Jacobian.
%       point_to_plane_residual - Compute the point to plane residual.
%       point_to_point_distance_jacobian - Compute the point to point distance Jacobian.
%       point_to_point_residual - Compute the point to point residual.
%       rotation_jacobian - Compute the rotation Jacobian.
%       translation_jacobian - Compute the translation Jacobian.
% See also DQ_SerialManipulator, DQ_MobileBase, DQ_CooperativeDualTaskSpace.

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
            % SET_REFERENCE_FRAME(reference_frame) sets the reference frame
            % used for the fkm() and pose_jacobian() methods
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
         % Given the Jacobian 'J_pose' and the corresponding unit dual
         % quaternion 'x_pose' that satisfy vec8(x_pose_dot) = J_pose *
         % q_dot, DISTANCE_JACOBIAN(J_pose,x_pose) returns the distance
         % Jacobian; that it, the Jacobian that satisfies the relation
         % dot(d^2) = Jd * q_dot, where dot(d^2) is the time derivative of
         % the square of the distance between the origin of the frame
         % represented by 'x_pose' and the origin of the reference frame.
             if ~is_unit(x_pose)
                error(['The second argument of distance_jacobian should be'...
                        ' a unit dual quaternion']);
             end
             p = translation(x_pose);
             Jp = DQ_Kinematics.translation_jacobian(J_pose,x_pose);
             Jd = 2*vec4(p)'*Jp;
        end
        
        function J = line_to_line_distance_jacobian(line_jacobian, ...
                                                    robot_line, workspace_line)
        % LINE_TO_LINE_DISTANCE_JACOBIAN(line_jacobian, robot_line,
        % workspace_line) returns the Jacobian 'J' that relates the joint
        % velocities (q_dot) to the time derivative of the square distance
        % between a line rigidly attached to the robot and a line in the
        % workspace.
        %
        % For more details, see Section IV.E of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also line_to_line_residual

            DOFS = size(line_jacobian,2);
            
            % Inner-product Jacobian (Eq. 39 of Marinho et al., 2019)
            Jinner = -0.5*(hamiplus8(workspace_line) + ...
                                        haminus8(workspace_line))*line_jacobian;
            
            % Jacobian of the square of the norm of the dual part of 
            % dot(robot_line, workspace_line)---Eq. 43 of Marinho et al., 2019.
            lz_inner_l = dot(robot_line,workspace_line);
            D_lz_inner_l = D(lz_inner_l);
            Jnorm_inner_dual = 2*vec4(D_lz_inner_l)'*Jinner(5:8,1:DOFS);

            % Cross product Jacobian (Eq. 40 of Marinho et al., 2019)
            Jcross = 0.5*(haminus8(workspace_line) - hamiplus8(workspace_line))...
                * line_jacobian;
            
            % Jacobian of the square of the norm of the primary part of
            % cross(robot_line, workspace_line)---Eq. 44 of Marinho et al.
            % (2019).
            lz_cross_l = cross(robot_line, workspace_line);
            P_lz_cross_l = P(lz_cross_l);
            Jnorm_cross_primary = 2*vec4(P_lz_cross_l)'*Jcross(1:4,1:DOFS);
            
            % Retrieve the angle between the lines---Eq. 35 of Marinho et
            % al. (2019)
            phi = acos(double(P(lz_inner_l)));

            % If robot_line and workspace_line are not parallel
            if mod(phi,pi)            
                % Non-parallel Distance Jacobian---Eq. 42 if Marinho et al. 
                % (2019)
                a = 1.0/(norm(P_lz_cross_l) * norm(P_lz_cross_l));
                b = -norm(D_lz_inner_l)*norm(D_lz_inner_l) * a * a;

                % Robot line--line squared distance Jacobian---Eq. 45 of
                % Marinho et al. (2019).
                J = double(a)*Jnorm_inner_dual + double(b)*Jnorm_cross_primary;
            else
                % robot_line and workspace_line are parallel
                D_lz_cross_l = D(lz_cross_l);
                % Eq. 47 of Marinho et al. (2019)
                J = 2*vec4(D_lz_cross_l)'*Jcross(5:8,1:DOFS);
            end
        end


        function residual = line_to_line_residual(robot_line, workspace_line,...
                workspace_line_derivative)
        % LINE_TO_LINE_RESIDUAL(robot_line, workspace_line,
        % workspace_line_derivative) returns the residual related to the
        % time derivative of the square distance between a line rigidly
        % attached to the robot and a moving line in the workspace that is
        % independent of the robot motion (i.e., which does not depend on
        % the robot joint velocities)
        %
        % For more details, see Section IV.E of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also line_to_line_distance_jacobian
           
            % Inner product residual---Eq. 39 of Marinho et al. (2019)
            zeta_lz_inner_l = dot(robot_line, workspace_line_derivative);
            D_zeta_lz_inner_l = D(zeta_lz_inner_l);
            
            % Residual of the square of the norm of the dual part of 
            % dot(robot_line, workspace_line)---Eq. 43 of Marinho et al., 2019.
            lz_inner_l = dot(robot_line,workspace_line);
            D_lz_inner_l = D(lz_inner_l);
            zeta_norm_inner_dual = 2*vec4(D_lz_inner_l)'*vec4(D_zeta_lz_inner_l);

            % Cross product residual---Eq. 40 of Marinho et al. (2019)
            zeta_lz_cross_l = cross(robot_line,workspace_line_derivative);
            P_zeta_lz_cross_l = P(zeta_lz_cross_l);
            D_zeta_lz_cross_l = D(zeta_lz_cross_l);
            
            % Residual of the square of the norm of the primary part of
            % cross(robot_line, workspace_line)
            lz_cross_l = cross(robot_line,workspace_line);
            zeta_norm_cross_primary = 2*vec4(lz_cross_l)'*vec4(P_zeta_lz_cross_l);
            
            % Retrieve the angle between the lines---Eq. 35 of Marinho et
            % al. (2019)
            phi = acos(double(P(lz_inner_l)));

            % If robot_line and workspace_line are not parallel
            if ~mod(phi,pi)            
                % Non-parallel Distance Jacobian---Eq. 42 if Marinho et al. 
                % (2019)
                a = 1.0/(norm(P_lz_cross_l) * norm(P_lz_cross_l));
                b = -norm(D_lz_inner_l)*norm(D_lz_inner_l) * a * a;
                
                residual = a*zeta_norm_inner_dual + b*zeta_norm_cross_primary;
            else
                % robot_line and workspace_line are parallel
                residual = 2*vec4(D(lz_cross_l))'*vec4(D_zeta_lz_cross_l);
            end 
        end
        
        function J = line_to_point_distance_jacobian(line_jacobian, ...
            robot_line, workspace_point)
        % LINE_TO_POINT_DISTANCE_JACOBIAN(line_jacobian, robot_line,
        % workspace_point) returns the Jacobian 'J' that relates the joint
        % velocities (q_dot) to the time derivative of the square distance
        % between a line in the robot and a point in in the workspace.
        %
        % For more details, see Eq. (34) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also line_to_point_residual
        
            n_columns = size(line_jacobian,2);

            Jl = line_jacobian(1:4, 1:n_columns);
            Jm = line_jacobian(5:8, 1:n_columns);

            % Extract line direction
            l = P(robot_line);
            % Extract line moment
            m = D(robot_line);
            
            h = cross(workspace_point,l) - m;

            J = 2*vec4(h)' * (crossmatrix4(workspace_point) * Jl - Jm);
        end

        function residual = line_to_point_residual(robot_line, ...
                workspace_point, workspace_point_derivative)
        % LINE_TO_POINT_RESIDUAL(robot_line, workspace_point,
        % workspace_point_derivative) returns the residual related to the
        % time derivative of the square distance between a line rigidly
        % attached to the robot and a moving point in the workspace that is
        % independent of the robot motion (i.e., which does not depend on
        % the robot joint velocities)
        %
        % For more details, see Eq. (34) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also line_to_point_distance_jacobian
        
            % Extract line quaternions
            l = P(robot_line);
            m = D(robot_line);

            % Notational simplicity
            hc1 = cross(workspace_point,l) - m;
            hc2 = cross(workspace_point_derivative,l);

            residual = double(2.0*dot(hc2,hc1));
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

            % Assemble the 8-by-n line Jacobian, where n =
            % dim_configuration_space
            Jlx = [Jrx; Jmx];
        end
        
        function Jr = rotation_jacobian(J_pose)
        % Given the Jacobian 'J_pose' and the corresponding unit dual
        % quaternion 'x_pose' that satisfy vec8(x_pose_dot) = J_pose *
        % q_dot, ROTATION_JACOBIAN(J_pose) returns the Jacobian Jr that
        % satisfies vec4(r_dot) = Jr * q_dot, where r_dot is the time
        % derivative of the rotation quaternion r in x_pose = r +
        % DQ.E*(1/2)*p*r and q_dot is the time derivative of the
        % configuration vector.
            Jr = J_pose(1:4,:);
        end
        
        function Jplane = plane_jacobian(pose_jacobian, x_pose, plane_normal)
        % PLANE_JACOBIAN(pose_jacobian, x_pose, plane_normal) returns the plane
        % jacobian, where x_pose is the end-effector pose, pose_jacobian is the 
        % matrix that satisfies x_pose_dot = pose_jacobian * q_dot, with q_dot 
        % being the joint velocities, and plane_normal is the plane normal with
        % respect to the reference frame. For example using i_, j_, and k_
        % will return the plane Jacobian whose normal is collinear with, 
        % respectively, the x-axis, y-axis, and z-axis of the end-effector
        % frame given by x_pose
        %
        % For more details, see Section IV.F of Marinho, M. M., Adorno, B. V., 
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
        
        function J = plane_to_point_distance_jacobian(plane_jacobian, ...
                workspace_point)
        % PLANE_TO_POINT_DISTANCE_JACOBIAN(plane_jacobian, workspace_point)
        % returns the Jacobian 'J' that relates the joint velocities
        % (q_dot) to the time derivative of the distance between a plane
        % attached to the robot and a point in the workspace.
        %
        % For more details, see Eq. (56) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also plane_to_point_residual
        
            if ~is_pure_quaternion(workspace_point)
                error('The argument %s should be a pure quaternion',inputname(2));
            end
        
            n_columns = size(plane_jacobian,2);
            
            % Break plane_jacobian into blocks corresponding to the plane
            % direction and plane distance
            Jnz = plane_jacobian(1:4, 1:n_columns);
            Jdz = plane_jacobian(5,1:n_columns);

            % Plane distance Jacobian---Eq. 56 of Marinho et al. (2019)
            J =  vec4(workspace_point)'*Jnz - Jdz;
        end

        function residual = plane_to_point_residual(robot_plane,...
                workspace_point_derivative)
        % PLANE_TO_POINT_RESIDUAL(robot_plane, workspace_point_derivative)
        % returns the residual related to the time derivative of the
        % distance between a plane rigidly attached to the robot and a
        % moving point in the workspace that is independent of the robot
        % motion (i.e., which does not depend on the robot joint
        % velocities)
        %
        % For more details, see Eq. (55) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also plane_to_point_distance_jacobian
            if ~is_plane(robot_plane) ||...
                    ~is_pure_quaternion(workspace_point_derivative)
                error(['The argument %s should be a plane and %s should be '...
                      'a pure quaternion'],inputname(1), inputname(2));
            end
            n_pi = P(robot_plane);
            residual = double(dot(workspace_point_derivative,n_pi));    
        end

        function J = point_to_line_distance_jacobian(translation_jacobian,...
                robot_point, workspace_line)
        % POINT_TO_LINE_DISTANCE_JACOBIAN(translation_jacobian,
        % robot_point, workspace_line) returns the Jacobian 'J' that
        % relates the joint velocities (q_dot) to the time derivative of
        % the square distance between a line in the workspace and a point
        % in in the robot.
        %
        % For more details, see Eq. (32) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also point_to_line_residual

            if ~is_pure_quaternion(robot_point_translation) || ...
                    ~is_line(workspace_line)
                error(['robot_point has to be a pure quaternion and'...
                    'workspace_line must be a pure dual quaternion']);
            end

            l = P(workspace_line);
            m = D(workspace_line);

            J = 2.0*vec4(cross(robot_point,l) - m)'*crossmatrix4(l)'* ...
               translation_jacobian;
        end

        function residual = point_to_line_residual(robot_point, workspace_line,...
                                workspace_line_derivative)
        % POINT_TO_LINE_RESIDUAL(robot_point, workspace_line,
        % workspace_line_derivative) returns the residual related to the
        % moving line in the workspace that is independent of the robot
        % motion (i.e., which does not depend on the robot joint
        % velocities)
        %
        % For more details, see Eq. (32) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also point_to_line_distance_jacobian

            if ~is_pure_quaternion(robot_point) || ~is_line(workspace_line) || ...
                    ~is_pure(workspace_line_derivative)
                error(['robot_point has to be a pure quaternion and'...
                    'both workspace_line and workspace_line_derivative must '...
                    'be pure dual quaternions']);
            end
           
            l = P(workspace_line);
            m = D(workspace_line);
            l_dot = P(workspace_line_derivative);
            m_dot = D(workspace_line_derivative);
            
            % See Eq. (30) in Marinho et al. (2018)
            hA1 = cross(robot_point,l) - m;
            hA2 = cross(robot_point,l_dot) - m_dot;
            residual = double(2*dot(hA2,hA1));
        end
        
        function J = point_to_plane_distance_jacobian(translation_jacobian, ...
                robot_point, workspace_plane)
        % POINT_TO_PLANE_DISTANCE_JACOBIAN(translation_jacobian,
        % robot_point, workspace_plane) returns the Jacobian 'J' that
        % relates the joint velocities (q_dot) to the time derivative of
        % the distance between a plane in the workspace and a point in in
        % the robot.
        %
        % For more details, see Eq. (59) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also point_to_plane_residual

            if ~is_pure_quaternion(robot_point) || ~is_plane(workspace_plane)
                error('%s must be a pure quaternion and %s must be a plane',...
                    inputname(2), inputname(3));
            end
            n = P(workspace_plane);
            J = vec4(n)'*translation_jacobian;
        end

        function residual = point_to_plane_residual(point, plane_derivative)
        % POINT_TO_PLANE_RESIDUAL(point, plane_derivative) returns the
        % residual related to the moving plane in the workspace that is
        % independent of the robot motion (i.e., which does not depend on
        % the robot joint velocities)
        %
        % For more details, see Eq. (59) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also point_to_plane_distance_jacobian
            if ~is_pure_quaternion(point) ||...
                ~(is_pure_quaternion(P(plane_derivative)) ...
                                                && is_real(D(plane_derivative)))
                error(['The argument %s has to be pure and %s must be the '...
                       'plane derivative; that is, it must have a pure '...
                       'quaternion in the primary part and a real number in '...
                       'the dual part'],inputname(point));
            end
            
            n_dot = P(plane_derivative);
            d_dot = D(plane_derivative);

            residual =  double(dot(point,n_dot) - d_dot);
        end
        
        function J = point_to_point_distance_jacobian(translation_jacobian,...
                robot_point, workspace_point)
        % POINT_TO_POINT_DISTANCE_JACOBIAN(translation_jacobian,
        % robot_point, workspace_point) returns the Jacobian 'J' that
        % relates the joint velocities (q_dot) to the time derivative of
        % the square distance between a point in the workspace and a point
        % in in the robot.
        %
        % For more details, see Eq. (22) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also point_to_point_residual

            if ~is_pure_quaternion(robot_point) ||...
                    ~is_pure_quaternion(workspace_point)
    
                error(['The arguments %s and %s have to be'...
                    'pure quaternions.',inputname(2), inputname(3)]);
            end
            J = 2 * vec4(robot_point - workspace_point)'*translation_jacobian;
        end

        function residual = point_to_point_residual(robot_point,...
                workspace_point, workspace_point_derivative)
        % POINT_TO_POINT_RESIDUAL(robot_point, workspace_point,
        % workspace_point_derivative) returns the residual related to the
        % moving point in the workspace that is independent of the robot
        % motion (i.e., which does not depend on the robot joint
        % velocities)
        %
        % For more details, see Eq. (22) of Marinho, M. M., Adorno, B. V., 
        % Harada, K., and Mitsuishi, M. (2018). Dynamic Active Constraints for 
        % Surgical Robots using Vector Field Inequalities. 
        % http://arxiv.org/abs/1804.11270 
        %
        % See also point_to_point_distance_jacobian
            if ~is_pure_quaternion(robot_point) || ...
               ~is_pure_quaternion(workspace_point) || ...
               ~is_pure_quaternion(workspace_point_derivative)
               error('All arguments must be pure quaternions.');
            end
            tmp = 2.0*dot(robot_point-workspace_point, ...
                                            -1.0*workspace_point_derivative);
            residual = double(tmp);
         end
        
        function Jp = translation_jacobian(J_pose,x_pose)
        % Given the Jacobian 'J_pose' and the corresponding unit dual
        % quaternion 'x_pose' that satisfy vec8(x_pose_dot) = J_pose *
        % q_dot, TRANSLATION_JACOBIAN(J_pose,x_pose) returns the Jacobian
        % that satisfies the relation vec4(p_dot) = Jp * q_dot, where p_dot
        % is the time derivative of the translation quaternion p and q_dot
        % is the time derivative of the configuration vector
            if ~is_unit(x_pose)
                error(['The second argument of translation_jacobian should be'...
                    ' a unit dual quaternion']);              
            end
            Jp = 2*haminus4(x_pose.P')*J_pose(5:8,:)+2*hamiplus4(x_pose.D)*DQ.C4*J_pose(1:4,:);
        end
    end
end
