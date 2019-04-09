% CLASS DQ_kinematics
% Usage: robot_kine = DQ_kinematics(A,convention)
% - 'A' is a 4 x n matrix containing the Denavit-Hartenberg parameters
%   (n is the number of links)
%    A = [theta1 ... thetan;
%            d1  ...   dn;
%            a1  ...   an;
%         alpha1 ... alphan]
% - 'convention' is the convention used for the D-H parameters. Accepted
%    values are 'standard' and 'modified'
%
% The first row of 'A' contains the joint offsets. More specifically,
% theta_i is the offset for the i-th joint.
%
% Type DQ_kinematics.(method,property) for specific help.
% Ex.: help DQ_kinematics.fkm
%
% For more information about the available methods, see also
% Concrete:
%       get_dim_configuration_space
%       fkm
%       pose_jacobian
%       pose_jacobian_derivative
%       raw_fkm
%       raw_pose_jacobian
%       set_effector
% Concrete (Inherited from DQ_Kinematics):
%       set_base_frame
%       set_reference_frame
% Static (Inherited from DQ_Kinematics):
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

% TODO: Remove the virtual joints. Instead of helping, they cause a lot of
% confusion, specially among those trying to learn the library. The
% affected methods are: FKM and Jacobian.

classdef DQ_kinematics < DQ_Kinematics
    properties
        n_links;
        theta,d,a,alpha;
        % Dummy and n_dummy are deprecated and will be removed in the near
        % future
        dummy;
        n_dummy;
        convention;        
        effector;
        
        % Properties for the plot function        
        plotopt
        lineopt
    end
    
    properties (SetAccess = private)   
        % Handle used to access the robot's graphics information. It's used
        % mainly in the plot function.
        handle
    end
    
    methods
        function obj = DQ_kinematics(A,type)
            if nargin == 0
                error('Input: matrix whose columns contain the DH parameters')
            end
            
            obj.n_dummy = 0;
            if size(A,1) == 5
                %There are dummy joints
                obj.dummy = A(5,:);
                obj.n_dummy = sum(obj.dummy == 1);
                
            else
                obj.dummy = zeros(1,size(A,2));
            end
            
            obj.n_links = size(A,2);
            obj.theta = A(1,:);
            obj.d = A(2,:);
            obj.a = A(3,:);
            obj.alpha = A(4,:);
            
            obj.reference_frame = DQ(1); %Default base's pose
            obj.base_frame = DQ(1);
            obj.effector = DQ(1); %Default effector's pose
            
            % Define a unique robot name
            obj.name = sprintf('%f',rand(1));
            
            if nargin==1
                obj.convention='standard';
            else
                obj.convention=type;
            end
            
            %For visualisation
            obj.lineopt = {'Color', 'black', 'Linewidth', 2};            
            obj.plotopt = {};
            
        end
        
        function ret = get_dim_configuration_space(obj)
            ret = obj.n_links;
        end
        
        function set_effector(obj,effector)
            % SET_EFFECTOR(effector) sets the pose of the effector
            
            obj.effector = DQ(effector);
        end
        
        function x = raw_fkm(obj,q, ith)
            %   RAW_FKM(q) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   last joint (the displacements due to the base and the effector 
            %   are not taken into account).
            %
            %   'q' is the vector of joint variables
            %
            %   This is an auxiliary function to be used mainly with the
            %   Jacobian function.
            
            if nargin == 3
                n = ith;
            else
                n = obj.n_links;
            end
            
            if length(q) ~= (obj.n_links - obj.n_dummy)
                error('Incorrect number of joint variables');
            end
            
            x = DQ(1);
            
            j = 0;
            for i=1:n
                if obj.dummy(i) == 1
                    % The offset is taken into account inside the method dh2dq
                    x = x*dh2dq(obj,0,i);
                    j = j + 1;
                else
                    x = x*dh2dq(obj,q(i-j),i);
                end
            end
        end
        
        function x = fkm(obj,q, ith)
            %   FKM(q) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   end-effector pose. This function takes into account the
            %   displacement due to the base's and effector's poses.
            %
            %   'q' is the vector of joint variables
            %
            %   FKM(q, ith) calculates the FKM up to the ith link.
            %   If ith is the last link, it DOES NOT take into account the
            %   trasformation given by set_effector. If you want to take
            %   into account that transformation, use FKM(q)
            %   instead.
            
            if nargin == 3
                x = obj.reference_frame*obj.raw_fkm(q, ith); %Takes into account the base displacement
            else
                x = obj.reference_frame*obj.raw_fkm(q)*obj.effector;
            end
        end
        
        function dq = dh2dq(obj,theta,i)
            %   For a given link's DH parameters, calculate the correspondent dual
            %   quaternion
            %   Usage: dq = dh2dq(theta,i), where
            %          theta: joint angle
            %          i: link number
            
            if nargin ~= 3
                error('Wrong number of arguments. The parameters are theta and the correspondent link')
            end
            d = obj.d(i);
            a = obj.a(i);
            alpha = obj.alpha(i);
            
            if strcmp(obj.convention,'standard')
                h(1) = cos((theta+obj.theta(i))/2)*cos(alpha/2);
                h(2) = cos((theta+obj.theta(i))/2)*sin(alpha/2);
                h(3) = sin((theta+obj.theta(i))/2)*sin(alpha/2);
                h(4) = sin((theta+obj.theta(i))/2)*cos(alpha/2);
                d2 = d/2;
                a2 = a/2;
                
                
                h(5) = -d2*h(4)-a2*h(2);
                h(6) = -d2*h(3)+a2*h(1);
                h(7) = d2*h(2)+a2*h(4);
                h(8) = d2*h(1)-a2*h(3);
            else
                h1 = cos((theta+obj.theta(i))/2)*cos(alpha/2);
                h2 = cos((theta+obj.theta(i))/2)*sin(alpha/2);
                h3 = sin((theta+obj.theta(i))/2)*sin(alpha/2);
                h4 = sin((theta+obj.theta(i))/2)*cos(alpha/2);
                h(1) = h1;
                h(2) = h2;
                h(3) = -h3;
                h(4) = h4;
                d2 = d/2;
                a2 = a/2;
                
                h(5) = -d2*h4-a2*h2;
                h(6) = -d2*h3+a2*h1;
                h(7) = -(d2*h2+a2*h4);
                h(8) = d2*h1-a2*h3;
            end
            
            dq = DQ(h);
        end
        
        function p = get_z(~,h)
            p(1) = 0;
            p(2) = h(2)*h(4) + h(1)*h(3);
            p(3) = h(3)*h(4) - h(1)* h(2);
            p(4) = (h(4)^2-h(3)^2-h(2)^2+h(1)^2)/2;
            p(5) = 0;
            p(6) = h(2)*h(8)+h(6)*h(4)+h(1)*h(7)+h(5)*h(3);
            p(7) = h(3)*h(8)+h(7)*h(4)-h(1)*h(6)-h(5)*h(2);
            p(8) = h(4)*h(8)-h(3)*h(7)-h(2)*h(6)+h(1)*h(5);
        end
        
        function J = raw_pose_jacobian(obj,q,ith)
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
            
            if nargin == 3
                n = ith;
                x_effector = obj.raw_fkm(q,ith);
            else
                n = obj.n_links;
                x_effector = obj.raw_fkm(q);
            end
            
            x = DQ(1);
            J = zeros(8,n-obj.n_dummy);
            jth=0;
            
            for i = 0:n-1
                % Use the standard DH convention
                if strcmp(obj.convention,'standard')
                    z = DQ(obj.get_z(x.q));
                else % Use the modified DH convention
                    w = DQ([0,0,-sin(obj.alpha(i+1)),cos(obj.alpha(i+1)),0, ...
                        0,-obj.a(i+1)*cos(obj.alpha(i+1)), ...
                        -obj.a(i+1)*sin(obj.alpha(i+1))] );
                    z = 0.5*x*w*x';
                end
                
                if ~obj.dummy(i+1)
                    x = x*obj.dh2dq(q(jth+1),i+1);
                    j = z * x_effector;
                    J(:,jth+1) = j.q;
                    jth = jth+1;
                else
                    % Dummy joints don't contribute to the Jacobian
                    x = x*obj.dh2dq(0,i+1);
                end
            end
        end
        
        function J = pose_jacobian(obj, q, ith)
            % POSE_JACOBIAN(q) returns the Jacobian that satisfies
            % vec(x_dot) = J * q_dot, where x = fkm(q) and
            % q is the vector of joint variables. It takes into account
            % both base and end-effector displacements (their default
            % values are 1).
            
            if nargin == 3 && ith < obj.n_links
                % If the Jacobian is not related to the mapping between the
                % end-effector velocities and the joint velocities, it takes
                % into account only the base displacement
                J = hamiplus8(obj.reference_frame)*obj.raw_pose_jacobian(...
                    q, ith);
            else
                % Otherwise, it the Jacobian is related to the
                % end-effector velocity, it takes into account both base
                % and end-effector (constant) displacements.
                J = hamiplus8(obj.reference_frame)*haminus8(obj.effector)*...
                    obj.raw_pose_jacobian(q);
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
            
            if nargin == 4
                n = ith;
                x_effector = obj.raw_fkm(q,ith);
                J = obj.raw_pose_jacobian(q,ith);
                vec_x_effector_dot = J*q_dot(1:ith);
            else
                n = obj.n_links;
                x_effector = obj.raw_fkm(q);
                J = obj.raw_pose_jacobian(q);
                vec_x_effector_dot = J*q_dot;
            end
                                 
            x = DQ(1);            
            J_dot = zeros(8,n-obj.n_dummy);
            jth = 0;
            
            for i = 0:n-1
                % Use the standard DH convention
                if strcmp(obj.convention,'standard')
                    w = DQ.k;
                    z = DQ(obj.get_z(x.q));
                else % Use the modified DH convention
                    w = DQ([0,0,-sin(obj.alpha(i+1)),cos(obj.alpha(i+1)),0, ...
                        0,-obj.a(i+1)*cos(obj.alpha(i+1)),...
                        -obj.a(i+1)*sin(obj.alpha(i+1))] );
                    z = 0.5*x*w*x';
                end
                
                if ~obj.dummy(i+1)                   
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
                    J_dot(:,jth+1) = haminus8(x_effector)*vec_zdot +...
                        hamiplus8(z)*vec_x_effector_dot;
                    x = x*obj.dh2dq(q(jth+1),i+1);
                    jth = jth+1;
                else
                    % Dummy joints don't contribute to the Jacobian
                    x = x*obj.dh2dq(0,i+1);                    
                end
            end
        end
        
        %% Deprecated methods. They will be removed in the near future.
        function set_base(obj,frame)
            warning(['The function set_base() is deprecated and will be '...
                'removed in the future. Please use set_reference_frame() '...
                'instead']);
            obj.set_reference_frame(frame);
        end
            
        
        function J = jacobian(obj,theta, ith)
            warning(['The function jacobian() is deprecated and will be '...
                'removed in the future. Please use pose_jacobian() '...
                'instead']);
            if nargin == 3
                J = pose_jacobian(obj,theta, ith);
            else
                J = pose_jacobian(obj,theta);
            end
        end        
        
        function J_dot = jacobian_dot(obj,theta,theta_dot, ith)
            warning(['The function jacobian_dot is deprecated and will be '...
                'removed in the future. Please use pose_jacobian_derivative() '...
                'instead']);
            if nargin == 4
                J_dot = pose_jacobian_derivative(obj,theta,theta_dot, ith);
            else
                J_dot = pose_jacobian_derivative(obj,theta,theta_dot);
            end
        end
        
        function ret = links(obj)
            warning(['The function links is deprecated and will be '...
                'removed in the future. Please use n_links '...
                'instead']);
            ret = obj.n_links;
        end
            
        
    end
    
%% Deprecated static methods. They will be removed in the near future.    
    
    methods(Static)
        function ret = C4()
             warning(['DQ_kinematics.C4 is deprecated and will be removed'... 
                'in the future. Please use DQ.C4 instead']);
            ret = DQ.C4;
        end
        
        function ret = C8()
             warning(['DQ_kinematics.C8 is deprecated and will be removed'... 
                'in the future. Please use DQ.C8 instead']);
            ret = DQ.C8;
        end
        
        function Jp = jacobp(J,x)
            warning(['The function jacobp is deprecated and will be removed'... 
                'in the future. Please use position_jacobian() instead']);
            Jp = DQ_kinematics.translation_jacobian(J,x);
        end
        
        function Jd = jacobd(J,x)
            warning(['The function jacobd is deprecated and will be removed'... 
                'in the future. Please use distance_jacobian() instead']);
            Jd = DQ_kinematics.distance_jacobian(J,x);
        end
    end
end