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
% METHODS:
%       raw_fkm
%       fkm
%       jacobian
%       jacobian_dot
%       jacobp
%       jacobd
%       set_base
%       set_effector
%
% PROPERTIES:
%       C8
%       C4
% See also DQ, DQ_KinematicController

% (C) Copyright 2015 DQ Robotics Developers
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
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br

% TODO: Remove the virtual joints. Instead of helping, they cause a lot of
% confusion, specially among those trying to learn the library. The
% affected methods are: FKM and Jacobian.

classdef DQ_kinematics < handle
    properties
        links;
        theta,d,a,alpha;
        dummy;
        n_dummy;
        convention;
        base;
        effector;
        
        % Properties for interfacing with Robotics Toolbox. Those are
        % legacy properties and are likely to disappear in the near
        % future.
        name;
        plotopt
        lineopt
        shadowopt
        robot_RT;
    end
    
    properties (SetAccess = private)
        
        handle
        %TODO: rename this property to avoid confusion with robot
        %configuration
        q
        
    end
    
    
    %TODO: Re-evaluate the need of redefining these properties below as
    %      they are already declared in DQ.
    properties (Constant)
        % Given the jacobian matrix J that satisfies
        % vec(dot_x) = J * dot_theta, where x is a dual quaternion, the
        % following relation is established:
        % vec(dot_x') = C8 * J * dot_theta, where x' is the conjugate of x.
        C8 = diag([1, -1, -1, -1, 1, -1, -1, -1]);
        % Given the jacobian matrix J that satisfies
        % vec(dot_r) = J * dot_theta, where r is a quaternion, the following
        % relation is established: vec(dot_r') = C4 * J * dot_theta, where
        % r' is the conjugate of r.
        C4 = diag([1, -1, -1, -1]);
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
            
            obj.links = size(A,2);
            obj.theta = A(1,:);
            obj.d = A(2,:);
            obj.a = A(3,:);
            obj.alpha = A(4,:);
            
            obj.base = DQ(1); %Default base's pose
            obj.effector = DQ(1); %Default effector's pose
            
            %Definitions for Robotics Toolbox
            obj.name = sprintf('%f',rand(1));
            
            if nargin==1
                obj.convention='standard';
            else
                obj.convention=type;
            end
            
            %For visualisation
            obj.lineopt = {'Color', 'black', 'Linewidth', 4};
            obj.shadowopt = {'Color', 0.7*[1 1 1], 'Linewidth', 3};
            obj.plotopt = {};
            
        end
        
        function set_base(obj,base)
            % dq.set_base(base) sets the pose of the robot's base
            
            obj.base = DQ(base);
        end
        
        function set_effector(obj,effector)
            % dq.set_effector(effector) sets the pose of the effector
            
            obj.effector = DQ(effector);
        end
        
        
        
        
        function q = raw_fkm(obj,theta, ith)
            %   dq = raw_fkm(theta) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   last joint (the displacements due to the base and the effector 
            %   are not taken into account).
            %   theta is the vector of joint variables
            %   This is an auxiliary function to be used mainly with the
            %   Jacobian function.
            
            if nargin == 3
                n = ith;
            else
                n = obj.links;
            end
            
            if length(theta) ~= (obj.links - obj.n_dummy)
                error('Incorrect number of joint variables');
            end
            
            q=DQ(1);
            
            j = 0;
            for i=1:n
                if obj.dummy(i) == 1
                    %The offset is taken into account inside the method dh2dq
                    q = q*dh2dq(obj,0,i);
                    j = j + 1;
                else
                    q = q*dh2dq(obj,theta(i-j),i);
                end
            end
        end
        
        function q = fkm(obj,theta, ith)
            %   dq = fkm(theta) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   end-effector pose. This function takes into account the
            %   displacement due to the base's and effector's poses.
            %
            %   theta is the vector of joint variables
            %
            %   dq = fkm(theta, ith) calculates the FKM up to the ith link.
            
            if nargin == 3
                q = obj.base*obj.raw_fkm(theta, ith); %Takes into account the base displacement
            else
                q = obj.base*obj.raw_fkm(theta)*obj.effector;
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
                q(1)=cos((theta+obj.theta(i))/2)*cos(alpha/2);
                q(2)=cos((theta+obj.theta(i))/2)*sin(alpha/2);
                q(3)=sin((theta+obj.theta(i))/2)*sin(alpha/2);
                q(4)=sin((theta+obj.theta(i))/2)*cos(alpha/2);
                d2=d/2;
                a2=a/2;
                
                
                q(5)=-d2*q(4)-a2*q(2);
                q(6)=-d2*q(3)+a2*q(1);
                q(7)=d2*q(2)+a2*q(4);
                q(8)=d2*q(1)-a2*q(3);
            else
                h1 = cos((theta+obj.theta(i))/2)*cos(alpha/2);
                h2 = cos((theta+obj.theta(i))/2)*sin(alpha/2);
                h3 = sin((theta+obj.theta(i))/2)*sin(alpha/2);
                h4 = sin((theta+obj.theta(i))/2)*cos(alpha/2);
                q(1)= h1;
                q(2)= h2;
                q(3)= -h3;
                q(4)= h4;
                d2=d/2;
                a2=a/2;
                
                
                q(5)=-d2*h4-a2*h2;
                q(6)=-d2*h3+a2*h1;
                q(7)=-(d2*h2+a2*h4);
                q(8)=d2*h1-a2*h3;
            end
            
            dq=DQ(q);
        end
        
        function p = get_z(obj,q)
            p(1) = 0;
            p(2)=q(2)*q(4) + q(1)*q(3);
            p(3)=q(3)*q(4) - q(1)* q(2);
            p(4)=(q(4)^2-q(3)^2-q(2)^2+q(1)^2)/2;
            p(5)=0;
            p(6)=q(2)*q(8)+q(6)*q(4)+q(1)*q(7)+q(5)*q(3);
            p(7)=q(3)*q(8)+q(7)*q(4)-q(1)*q(6)-q(5)*q(2);
            p(8)=q(4)*q(8)-q(3)*q(7)-q(2)*q(6)+q(1)*q(5);
        end
        
        function J = raw_jacobian(obj,theta,ith)
            % J = raw_jacobian(theta) returns the Jacobian that satisfies 
            % vec(x_dot) = J * theta_dot, where x = fkm(theta) and theta is the 
            % vector of joint variables.
            %
            % J = raw_jacobian(theta,ith) returns the Jacobian that
            % satisfies vec(x_ith_dot) = J * theta_dot(1:ith), where 
            % x_ith = fkm(theta, ith), that is, the fkm up to the i-th link.
            %
            % This function does not take into account any base or
            % end-effector displacements and should be used mostly
            % internally in DQ_kinematics
            
            if nargin == 3
                n = ith;
                x_effector = obj.raw_fkm(theta,ith);
            else
                n = obj.links;
                x_effector = obj.raw_fkm(theta);
            end
            
            x = DQ(1);
            J= zeros(8,n-obj.n_dummy);
            jth=0;
            
            for i = 0:n-1
                % Use the standard DH convention
                if strcmp(obj.convention,'standard')
                    z = DQ(obj.get_z(x.q));
                else % Use the modified DH convention
                    w = DQ([0,0,-sin(obj.alpha(i+1)),cos(obj.alpha(i+1)),0,0,-obj.a(i+1)*cos(obj.alpha(i+1)),-obj.a(i+1)*sin(obj.alpha(i+1)) ] );
                    z = 0.5*x*w*x';
                end
                
                if ~obj.dummy(i+1)
                    x = x*obj.dh2dq(theta(jth+1),i+1);
                    j = z * x_effector;
                    J(:,jth+1) = j.q;
                    jth = jth+1;
                else
                    % Dummy joints don't contribute to the Jacobian
                    x = x*obj.dh2dq(0,i+1);
                end
            end
        end
        
        
        function J = jacobian(obj, theta, ith)
            % J = jacobian(theta) returns the Jacobian that satisfies
            % vec(x_dot) = J * theta_dot, where x = fkm(theta) and
            % theta is the vector of joint variables. It takes into account
            % both base and end-effector displacements (their default
            % values are 1).
            
            if nargin == 3 && ith < obj.links
                % If the Jacobian is not related to the mapping between the
                % end-effector velocities and the joint velocities, it takes
                % into account only the base displacement
                J = hamiplus8(obj.base)*obj.raw_jacobian(theta, ith);
            else
                % Otherwise, it the Jacobian is related to the
                % end-effector velocity, it takes into account both base
                % and end-effector (constant) displacements.
                J = hamiplus8(obj.base)*haminus8(obj.effector)*obj.raw_jacobian(theta);
            end
        end
        
        
        
        function J_dot = jacobian_dot(obj,theta,theta_dot, ith)
            % J_dot = jacobian_dot(theta,theta_dot) returns the Jacobian 
            % time derivative.
            % J_dot = jacobian_dot(theta,theta_dot,ith) returns the first
            % ith columns of the Jacobian time derivative.
            % This function does not take into account any base or
            % end-effector displacements.
            
            if nargin == 4
                n = ith;
                x_effector = obj.raw_fkm(theta,ith);
                J = obj.raw_jacobian(theta,ith);
                vec_x_effector_dot = J*theta_dot(1:ith);
            else
                n = obj.links;
                x_effector = obj.raw_fkm(theta);
                J = obj.raw_jacobian(theta);
                vec_x_effector_dot = J*theta_dot;
            end
                                 
            x = DQ(1);            
            J_dot = zeros(8,n-obj.n_dummy);
            jth=0;
            
            for i = 0:n-1
                % Use the standard DH convention
                if strcmp(obj.convention,'standard')
                    w = DQ.k;
                    z = DQ(obj.get_z(x.q));
                else % Use the modified DH convention
                    w = DQ([0,0,-sin(obj.alpha(i+1)),cos(obj.alpha(i+1)),0,0,-obj.a(i+1)*cos(obj.alpha(i+1)),-obj.a(i+1)*sin(obj.alpha(i+1)) ] );
                    z = 0.5*x*w*x';
                end
                
                if ~obj.dummy(i+1)                   
                    % When i = 0 and length(theta) = 1, theta(1,i) returns
                    % a 1 x 0 vector, differently from the expected
                    % behavior, which is to return a 0 x 1 matrix.
                    % Therefore, we have to deal with the case i = 0
                    % explictly.
                    if i ~= 0
                        vec_zdot = 0.5*(haminus8(w*x') + hamiplus8(x*w)*DQ.C8)*obj.raw_jacobian(theta,i)*theta_dot(1:i);
                    else
                        vec_zdot = zeros(8,1);
                    end
                    J_dot(:,jth+1) = haminus8(x_effector)*vec_zdot + hamiplus8(z)*vec_x_effector_dot;
                    x = x*obj.dh2dq(theta(jth+1),i+1);
                    jth = jth+1;
                else
                    % Dummy joints don't contribute to the Jacobian
                    x = x*obj.dh2dq(0,i+1);                    
                end
            end
        end
        
    end
    
    methods(Static)
        
        function Jp = jacobp(J,x)
            %Given the dual quaternion Jacobian J and the corresponding
            %dual quaternion, Jp = jacobp(J,x) returns the translation Jacobian; that it,
            %the Jacobian that satisfies the relation dot_p = Jp * dot_theta,
            %where dot_p is the time derivative of the
            %translation quaternion and dot_theta is the time derivative of
            %the joint vector.
            x = DQ(x);
            Jp = 2*haminus4(x.P')*J(5:8,:)+2*hamiplus4(x.D)*DQ_kinematics.C4*J(1:4,:);
        end
        
        function Jd = jacobd(J,x)
            %Given the dual quaternion Jacobian J and the corresponding
            %dual quaternion, Jd = jacobd(J,x) returns the distance Jacobian; that it,
            %the Jacobian that satisfies the relation dot_(d^2) = Jd * dot_theta,
            %where dot_(d^2) is the time derivative of the
            %square of the distance between the end-effector and the base and dot_theta is the time derivative of
            %the joint vector.
            x = DQ(x);
            p = translation(x);
            Jp = DQ_kinematics.jacobp(J,x);
            Jd=2*vec4(p)'*Jp;
        end
    end
end