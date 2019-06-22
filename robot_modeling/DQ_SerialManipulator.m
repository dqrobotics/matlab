% Concrete class that defines serial manipulators.
%
% Usage: robot = DQ_SerialManipulator(A,convention)
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
% DQ_SerialManipulator Methods (Concrete):
%       get_dim_configuration_space - Return the dimension of the configuration space.
%       fkm - Compute the forward kinematics while taking into account base and end-effector's rigid transformations.
%       plot - Plots the serial manipulator.
%       pose_jacobian - Compute the pose Jacobian while taking into account base's and end-effector's rigid transformations.
%       pose_jacobian_derivative - Compute the time derivative of the pose Jacobian.
%       raw_fkm - Compute the FKM without taking into account base's and end-effector's rigid transformations.
%       raw_pose_jacobian - Compute the pose Jacobian without taking into account base's and end-effector's rigid transformations.
%       set_effector - Set an arbitrary end-effector rigid transformation with respect to the last frame in the kinematic chain.
% See also DQ_Kinematics.

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

classdef DQ_SerialManipulator < DQ_Kinematics
    properties        
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
    
    properties (SetAccess = protected)   
        % Handle used to access the robot's graphics information. It's used
        % mainly in the plot function.
        handle
        n_links;
    end
    
    methods
        function obj = DQ_SerialManipulator(A,type)
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
        
        function plot(robot,q,varargin)
        % plot(robot,q,options) plots the robot of type DQ_kinematics.
        % q is the vector of joint configurations
        % options is an optional argument that has variable size and accept any
        % number of the following pairs:
        %
        %  'workspace', W          size of robot 3D workspace, where
        %                          W = [xmn, xmx ymn ymx zmn zmx]
        %  'cylinder', C           color for joint cylinders, C=[r g b]
        %  'scale', scale          annotation scale factor
        %  'base'|'nobase'         controls display of base plane
        %  'wrist'|'nowrist'       controls display of wrist
        %  'name'|'noname'         display the robot's name
        %  'xyz'|'noa'             wrist axis label
        %  'joints'|'nojoints'     controls display of joints
        %
        % The graphical robot object holds a copy of the robot object and
        % the graphical element is tagged with the robot's name (.name property).
        %
        % 1) Figure behavior:
        %
        % If no robot of this name is currently displayed then a robot will
        % be drawn in the current figure.  If hold is enabled (hold on) then the
        % robot will be added to the current figure.
        %
        % If the robot already exists then that graphical model will be found
        % and moved.
        %
        % 2) Multiple views of the same robot:
        %
        % If one or more plots of this robot already exist then these will all
        % be moved according to the argument 'q'.  All robots in all windows with
        % the same name will be moved.
        %
        % NOTE: Since each kinematic robot stores just one graphical handle,
        % if we want to plot the same robot in different views, we must declare
        % different robots with the same name. Otherwise, if just one robot is declared,
        % but plotted in different windows/views, the kinematic robot will store
        % the handle of the last view only. Therefore, only the last view will be
        % updated
        %
        % 3) Multiple robots in the same figure:
        %
        % Multiple robots (i.e., with different names) can be displayed in the same
        % plot, by using "hold on" before calls to plot(robot).
        %
        % 4) Graphical robot state:
        %
        % The configuration of the robot as displayed is stored in the DQ_kinematics
        % object and can be accessed by the read only object property 'q'.
        %
        % 5) Graphical annotations and options:
        %
        % The robot is displayed as a basic stick figure robot with annotations
        % such as:
        % - XYZ wrist axes and labels,
        % - joint cylinders,
        % which are controlled by options.
        %
        % The size of the annotations is determined using a simple heuristic from
        % the workspace dimensions.  This dimension can be changed by setting the
        % multiplicative scale factor using the 'scale' option.
            has_options = 0;
            if nargin < 2
                fprintf(['\nUsage: plot(robot, q,[options])'...
                    '\ntype ''help DQ_kinematics/plot'' for more information\n']);
                return;
            elseif nargin >= 3
                has_options = 1;
            end
            
            if ~isvector(q)
                error('The first argument must be the vector of joint configurations.');
            end
            
            % The joint configuration vector must be a row vector
            if ~isrow(q)
                q = q';
            end
            
            if has_options
                dq_kinematics_plot(robot,q, varargin{:});
            else
                dq_kinematics_plot(robot,q);
            end
        end
        
        %% Deprecated methods. They will be removed in the near future.
        function set_base(obj,frame)
            warning(['The function set_base() is deprecated and will be '...
                'removed in the future. Please use set_reference_frame() '...
                'instead']);
            obj.set_base_frame(frame);
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

% dq_kinematics_plot(robot, q, varargin) creates a new robot, if it does
% not exist, otherwise it updates all robots with the same name.
% 
% In case a robot is created, the function create_new_robot() provides a
% handle 'h' to the graphical robot. Then, for the h.robot object we 
% additionally: 
%   - save this new kinematic robot object as its UserData
%   - tag it with the name field from the robot object
%
%  This enables us to find all robots with a given name, in all figures,
%  and update them.
function dq_kinematics_plot(robot, q, varargin)
    % process options
    if (nargin > 2) && isstruct(varargin{1})
        % options is a struct. 
        opt = varargin{1};
    else
        % options is a list of options; we need to transform it to a struct
        opt = plot_options(robot, varargin);
    end

    % Virtual (dummy) joints will be removed in the future
    n = robot.n_links-robot.n_dummy;

    if length(q) ~= n
        error('Incorrect number of joints. The correct number is %d', n);
    end

    % get handles of all existing robot with the same name
    graphic_robot_handle = findobj('Tag', robot.name);
    
    % Condition to verify that no robot with this name exists
    condition1 = isempty(graphic_robot_handle) || isempty(get(gcf, 'Children'));
    % Condition to verify if hold is on and no robot of this name is in the 
    % current axes
    condition2 = ishold && isempty(findobj(gca, 'Tag', robot.name));
    if condition1 || condition2
        % no robot with this name exists
        h = create_new_robot(robot, opt);
        % save the handle in the robot object and attach it to the robot 
        % as user data. This way, the data will be available to the
        % update_robot() function.
        robot.handle = h;
        set(h.robot, 'Tag', robot.name);
        set(h.robot, 'UserData', robot);
        graphic_robot_handle = h.robot;        
    end

    % Update all robots with the same name. This is very useful when we
    % want to visualize the same robot from different points of view
    % Each element of graphic_robot_handle has a graphic line robot
    for r = graphic_robot_handle'    
        % Inside the graphic line robot, we store the 'real' robot. We use
        % it in order to plot the robot.
        rr = get(r, 'UserData');
        update_robot(rr, q);
        % save the joint angles away in all the graphical robots with the
        % same name
        rr.q = q;
    end
end

% h = create_new_robot(robot, opt) uses data from robot object and options
% to create a graphical robot.
%
% Returns a structure of handles to graphical objects.
%
% If current figure is empty, draw robot in it
% If current figure has hold on, add robot to it
% Otherwise, create new figure and draw robot in it.
%   
% The handle is composed of the following fields:
% h.mag             Robot scale
% h.robot           the line segment that represents the robot
% h.x               the line segment that represents the end-effector x-axis
% h.y               the line segment that represents the end-effector y-axis
% h.z               the line segment that represents the end-effector z-axis
% h.xt              text for the end-effector x-axis
% h.yt              text for the end-effector y-axis
% h.zt              text for the end-effector z-axis
% h.joint(i)        the i-th joint (cylinder for revolute joints)
% h.base_handle     the robot base plane 
% h.name_handle     the robot name
function h = create_new_robot(robot, opt)
    h.mag = opt.mag;

    if ~ishold
        % if current figure has hold on, then draw robot here
        % otherwise, create a new figure
        axis(opt.workspace);
    end
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    set(gca, 'SortMethod', 'depth');
    grid on

    % Draw a small plane representing the robot base
    if opt.base
        plane = robot.base_frame.'*DQ.k*robot.base_frame';
        % Since the plane is infinite, the DQ.plot function draws the part
        % closest to the origin of the reference frame. We first
        % consider the plane that passes through the origin and is aligned with 
        % the one that supports the base
        base_handle = plot(plane.P,'plane',opt.mag,'color','k');
        % We then translate the 'visible' plane to the base frame
        base_translation = vec3(translation(robot.base_frame));
        plane_vertices = get(base_handle, 'Vertices');
        for i = 1:3
            plane_vertices(:,i) = plane_vertices(:,i) + base_translation(i);
        end        
        set(base_handle, 'Vertices', plane_vertices);
        h.base_handle = base_handle;
    end
     
    % Write the robot name.
    if opt.name        
        b = vec3(translation(robot.base_frame));
        h.name_handle = text(b(1), b(2) - opt.mag, b(3), [' ' robot.name],...
            'FontAngle', 'italic','FontWeight', 'bold');
    end
    
    % create a line that we will subsequently modify using the function 
    % update_robot(). 
    h.robot = line(robot.lineopt{:});
    
    % create end-effector frame
    if opt.wrist,   
        h.x = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'red');
        h.y = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'green');
        h.z = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'blue');
        h.xt = text(0, 0, opt.wristlabel(1), 'FontWeight', 'bold',...
            'HorizontalAlignment', 'Center');
        h.yt = text(0, 0, opt.wristlabel(2), 'FontWeight', 'bold',...
            'HorizontalAlignment', 'Center');
        h.zt = text(0, 0, opt.wristlabel(3), 'FontWeight', 'bold',...
            'HorizontalAlignment', 'Center');

    end

    % Display cylinders (revolute each joint).
    for i = 1:robot.n_links
        if opt.joints
            %TODO: implement prismatic joints
            N = 8;
            % define the vertices of the unit cylinder with radius opt.mag/4
            % the z coordinates of the bottom and top faces are 0 and 1,
            % respectively. Furthermore, xc, yc, and zc have 2 rows: the
            % first corresponds to the bottom coordinates and the second to
            % the top coordinates.
            [xc,yc,zc] = cylinder(opt.mag/4, N);            
            
            % Scale the cylinder
            % when zc == 0, make it -opt.mag/2, except for the first joint
            if i~= 1
                zc(zc==0) = -opt.mag/2;
            end
            % when zc == 1, make it opt.mag/2
            zc(zc==1) = opt.mag/2;

            % Create vertex color data. Each vertex receive a RGB triplet.
            % Red value is stored in cdata(:,:,1), green in cdata(:,:,2)
            % and blue in cdata
            cdata = zeros(size(xc,1),size(xc,2),3);
            for j=1:3
                cdata(:,:,j) = opt.cylinder(j);
            end
            % render the surface
            h.joint(i) = surface(xc,yc,zc,cdata);
          
            % set the surfaces to be smooth and translucent
            set(h.joint(i), 'FaceColor', 'interp');
            set(h.joint(i), 'EdgeColor', 'none');
            set(h.joint(i), 'FaceAlpha', 0.7);

            % build a matrix of coordinates so we
            % can transform the cylinder in update_robot()
            % and hang it off the cylinder
            xyz = [xc(:)'; yc(:)'; zc(:)'; ones(1,2*N+2)];
            
            set(h.joint(i), 'UserData', xyz);
        end
    end
end


% update_robot(robot, q) moves an existing graphical robot to the configuration
% specified by the joint coordinates q. The parameter 'robot' is the
% kinematic robot, and graphics are defined by the handle structure robot.handle, 
% which stores the 'graphical robot' as robot.handle.robot.
function update_robot(robot, q)
    % Dummy (virtual) joints will be removed in the near future
    n = robot.n_links-robot.n_dummy;
    
    % Get the handle to the graphical robot. Since each kinematic robot
    % stores just one graphical handle, if we want to plot the same robot
    % in different views, we must declare different robots with the same
    % name. Otherwise, if just one robot is declared, but plotted in
    % different windows/views, the kinematic robot will store the handle of
    % the last view only.
    h = robot.handle;
    mag = h.mag;
    base = vec3(translation(robot.base_frame));
    
    % Initialize the vector containing the origin of each frame along the
    % kinematic chain
    x = zeros(n+1,1);
    y = zeros(n+1,1);
    z = zeros(n+1,1);
    % The first element corresponds to the origin of the base frame
    x(1) = base(1);
    y(1) = base(2);
    z(1) = base(3);

    % compute the link transforms, and record the origin of each frame
    % for the graphics update.    
    for j=1:n    
        t = vec3(translation(robot.base_frame*robot.raw_fkm(q,j)));
        x(j+1) = t(1);    
        y(j+1) = t(2);
        z(j+1) = t(3);
    end
    
    % Update the coordinates of each frame along the kinematic chain. This 
    % updates the line drawing that represents the robot kinematic chain.
    set(h.robot,'xdata', x, 'ydata', y, 'zdata', z);
   
    % display the joints as cylinders
    if isfield(h, 'joint')       
        for j=1:n
            % get coordinate data from the cylinder. The corresponding UserData 
            % is never updated. Therefore, we must translate and rotate
            % each cylinder according to the new frames along the kinematic
            % chain
            xyz = get(h.joint(j), 'UserData');
            
            %The joints are located at the beginning of each link
            fkm_j = robot.raw_fkm(q,j-1);
            
            for k = 1:size(xyz,2)
                % 1 + DQ.E*(1/2)*p, where p = xyz(1:3,k);
                cylinder_vertex = DQ([1;0;0;0;0;0.5*xyz(1:3,k)]);
                xyz(1:3,k) = vec3(translation(robot.base_frame*fkm_j*cylinder_vertex));
            end
            
            % Now that all cylinder vertices are transformed, update the
            % cylinder drawing. Transform xc, yc, and zc to matrices of
            % two rows and ncols columns (i.e., half the number of vertices
            % as each column stores the bottom and top vertices).
            ncols = size(xyz,2)/2;
            xc = reshape(xyz(1,:), 2, ncols);
            yc = reshape(xyz(2,:), 2, ncols);
            zc = reshape(xyz(3,:), 2, ncols);
            
            set(h.joint(j), 'Xdata', xc, 'Ydata', yc, 'Zdata', zc);
        end
    end

   
    % display the wrist axes and labels   
    % compute the wrist axes, based on final link transformation plus the
    % tool transformation.
    if isfield(h, 'x')
        % get the end-effector pose (considering the final transformation given
        % by set_end_effector()
        t = robot.base_frame*robot.raw_fkm(q)*robot.effector;
        t1 = vec3(translation(t));        
        
        % The following transformations use the Hamilton operators to
        % improve performance as Matlab is optimized for matrix operations.
        
        % Recall that, given a rotation t.P, the point transformation of any
        % point p is given by t.P*p*t.P'. In terms of Hamilton operators,
        % the transformation is given by 
        % vec4(t.P*p*t.P') = hamiplus4(t.P)*haminus4(t.P')*vec4(p).
        
        % H(1,2:4) is always 0.
        H = hamiplus4(t.P)*haminus4(t.P');
        
        % We can simplify the calculations as follows
        xv = t1 + H(2:4,2)*mag; % p = [0; mag; 0; 0]
        yv = t1 + H(2:4,3)*mag; % p = [0; 0; mag; 0]
        zv = t1 + H(2:4,4)*mag; % p = [0; 0; 0; mag]        
        
        % update the wrist axes       
        set(h.x,'xdata',[t1(1) xv(1)], 'ydata', [t1(2) xv(2)], ...
            'zdata', [t1(3) xv(3)]);
        set(h.y,'xdata',[t1(1) yv(1)], 'ydata', [t1(2) yv(2)], ...
             'zdata', [t1(3) yv(3)]);
        set(h.z,'xdata',[t1(1) zv(1)], 'ydata', [t1(2) zv(2)], ...
             'zdata', [t1(3) zv(3)]);
         
        % update the axes' name positions 
        set(h.xt, 'Position', xv);
        set(h.yt, 'Position', yv);
        set(h.zt, 'Position', zv);
    end
end


% o = plot_options(robot, options) returns an options structure. 
% 'robot' is the kinematic robot and 'options' is an array cell with the plot
% options.
function o = plot_options(robot, optin)
    % process a cell array of options and return a struct   
    % define all possible options and their default values
    o.joints = true; % Plot the joints
    o.wrist = true; % Plot the end-effector coordinates   
    o.base = true; % Plot a small plane perpendicular to the first joint
    o.wristlabel = 'xyz'; % Axes names. Another option is 'nsa'
    o.scale = 1; % Scale the drawing
    o.name = true; % Write the robot name
    o.cylinder = [0 0 0.7]; % The joint colors   
    o.workspace = [];  % Define the robot workspace. If not defined, a 
                       % heuristic is used to determine the robot workspace
                       % based on its size.

    % Plot options can be stored in the robot as a cell array. If this
    % information is available, we use it together with the options passed to 
    % the plot function.
    options = [robot.plotopt optin];
    
    % parse the options
    if ~isempty(options)
        [o,args] = parse_options(o, options);        
        if ~isempty(args)
            error(['Unknown options: ', args{:}]);
        end
    end

    % NOTE: The calculations below happen all the time. Maybe we should
    % store it in the graphical robot in order to do it just once.
    
    % simple heuristic to figure the maximum reach of the robot
    if isempty(o.workspace)
        reach = 0;
        for i=1:robot.n_links
            % Since the maximum reaching distance are given by the link offset 
            % and link length, we add them.
            reach = reach + abs(robot.a(i)) + abs(robot.d(i));
        end
        o.workspace = [-reach reach -reach reach -reach reach];      
    else
        reach = min(abs(o.workspace));
    end
    % The size of the joints will depend on the size of the workspace. This
    % can be adjusted by using the parameter 'scale'
    o.mag = o.scale * reach/15;
end

% [opt, others] = parse_options(default, options) parses the cell array inside
% 'options' and returns the corresponding structure 'opt'. The default
% parameters are given by 'default'.
%
% The software pattern is:
%       opt.foo = true;
%       opt.bar = false;
%       opt.blah = [];
%
% Optional arguments to the function behave as follows:
%   'foo'           sets opt.foo <- true
%   'nofoo'         sets opt.foo <- false
%   'blah', 3       sets opt.blah <- 3
%
% and can be given in any combination.
%
% NOTE:
% 1) The enumerator names must be distinct from the field names.
% 2) Only one value can be assigned to a field, if multiple values
%    are required they must be converted to a cell array.
%
% The allowable options are specified by the names of the fields in the
% structure opt.  By default if an option is given that is not a field of 
% opt an error is declared.
function [opt,others] = parse_options(default, options)

    arglist = {};

    argc = 1;
    opt = default;

    while argc <= length(options)
        current_option = options{argc};
      %  pause
        assigned = false;

        if ischar(current_option)
            % does the current_option match a field in the opt structure?            
            if isfield(opt, current_option)
                % If yes, then val = opt.(current_option)
                val = getfield(opt, current_option);
                
                %  In case the parameter is something like 'base', the 
                %  corresponding options field must be true. Therefore,
                %  opt.base = true
                if islogical(val)                   
                    opt = setfield(opt, current_option, true);
                else
                    % otherwise grab its value from the next arg
                    % opt.(current_option) = options{argc + 1}
                    opt = setfield(opt, current_option, options{argc+1});
                    argc = argc+1;
                end
                assigned = true;
            % The current option is a string, but does not correspond to an 
            % options field. We verify if the first two letters is 'no' and
            % the remainder of the string corresponds to an options field.
            % In this case, it means that we must assign 'false'to the
            % corresponding field.
            elseif length(current_option) > 2 && ...
                strcmp(current_option(1:2), 'no') && ...
                isfield(opt, current_option(3:end))
            
                val = getfield(opt, current_option(3:end));
              
                % We only update the corresponding field if its attribute
                % is a logical value. For example, 'nobase' implies opt.base
                % = false
                if islogical(val)
                    % a logical variable can only be set by an option
                    opt = setfield(opt, current_option(3:end), false);
                    assigned = true;
                end
            end           
        end
        
        % non matching options are collected arglist returns the unrecognized 
        % options.
        if ~assigned
            % This is a non-exhaustive list of invalid command types.
            if ~ischar(options{argc})                
                if isnumeric(options{argc})
                    numeric_string = num2str(options{argc});
                    arglist = [arglist, ' ''[', numeric_string,']'','];
                elseif iscell(options{argc})
                    arglist = [arglist, ' ''', '<INVALID CELL>',''','];
                end
            else
                arglist = [arglist, ' ''', options(argc),''','];
            end            
        end
        argc = argc + 1;
    end % while

    % if enumerator value not assigned, set the default value
    for field=fieldnames(default)'
        if iscell(getfield(default, field{1})) && iscell(getfield(opt, field{1}))
            val = getfield(opt, field{1});
            if isempty(val{1})
                opt = setfield(opt, field{1}, val{1});
            elseif val{1}(1) == '#'
                opt = setfield(opt, field{1}, 1);
            else
                opt = setfield(opt, field{1}, val{1});
            end
        end
    end 

    others = arglist;    
end

