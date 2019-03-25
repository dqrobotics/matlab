% plot(robot,q,options) plots the robot of type DQ_kinematics. 
% q is the vector of joint configurations
% options is an optional argument that has variable size and accept any
% number of the following pairs:
%
%  'workspace', W          size of robot 3D workspace, where
%                          W = [xmn, xmx ymn ymx zmn zmx]
%  'cylinder', C           color for joint cylinders, C=[r g b]
%  'mag', scale            annotation scale factor
%  'perspective'|'ortho'   type of camera view
%  'render'|'norender'     controls shaded rendering after drawing
%  'loop'|'noloop'         controls endless loop mode
%  'base'|'nobase'         controls display of base plane
%  'wrist'|'nowrist'       controls display of wrist
%  'shadow'|'noshadow'     controls display of shadow
%  'name'|'noname'         display the robot's name 
%  'xyz'|'noa'             wrist axis label
%  'joints'|'nojoints'     controls display of joints
%
% See also DQ/plot


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
%     
%     Part of this file was adapted from Peter Corke's Robotic Toolbox (2011 
%     version) - http://www.petercorke.com

function plot(robot,q,varargin)    
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

% The graphical robot object holds a copy of the robot object and
% the graphical element is tagged with the robot's name (.name property).
% This state also holds the last joint configuration which can be retrieved,
% see PLOT(robot) below.
%
% Figure behaviour::
% If no robot of this name is currently displayed then a robot will
% be drawn in the current figure.  If hold is enabled (hold on) then the
% robot will be added to the current figure.
%
% If the robot already exists then that graphical model will be found 
% and moved.
%
% Multiple views of the same robot::
%
% If one or more plots of this robot already exist then these will all
% be moved according to the argument Q.  All robots in all windows with 
% the same name will be moved.
%
% Multiple robots in the same figure::
%
% Multiple robots can be displayed in the same plot, by using "hold on"
% before calls to plot(robot).  
%
% Graphical robot state::
%
% The configuration of the robot as displayed is stored in the SerialLink object
% and can be accessed by the read only object property R.q.
%
% Graphical annotations and options::
%
% The robot is displayed as a basic stick figure robot with annotations 
% such as:
% - XYZ wrist axes and labels
% - joint cylinders and axes
% which are controlled by options.
%
% The size of the annotations is determined using a simple heuristic from 
% the workspace dimensions.  This dimension can be changed by setting the 
% multiplicative scale factor using the 'mag' option.
%

%
% The options come from 3 sources and are processed in order:
% - Cell array of options returned by the function PLOTBOTOPT.
% - Cell array of options given by the 'plotopt' option when creating the
%   SerialLink object.
% - List of arguments in the command line.
%
% See also plotbotopt, SerialLink.fkine.


% HANDLES:
%
%  A robot comprises a bunch of individual graphical elements and these are 
% kept in a structure which can be stored within the .handle element of a
% robot object:
%   h.robot     the robot stick figure
%   h.x     wrist vectors
%   h.y
%   h.z
%   h.xt        wrist vector labels
%   h.yt
%   h.zt
%
%  The plot function returns a new robot object with the handle element set.
%
% For the h.robot object we additionally: 
%   - save this new robot object as its UserData
%   - tag it with the name field from the robot object
%
%  This enables us to find all robots with a given name, in all figures,
% and update them.

function dq_kinematics_plot(robot, q, varargin)
    % process options
    if (nargin > 2) && isstruct(varargin{1})
        % options is a struct
        opt = varargin{1};
    else
        % options is a list of options
        opt = plot_options(robot, varargin);
    end

    % Virtual (dummy) joints will be removed in the future
    n = robot.links-robot.n_dummy;

    if length(q) ~= n
        error('Insufficient number of joints')
    end

    % get handles of all existing robot with the same name
    robot_handle = findobj('Tag', robot.name);
    
    % Condition to verify that no robot with this name exists
    condition1 = isempty(robot_handle) || isempty(get(gcf, 'Children'));
    % Condition to verify if hold is on and no robot of this name is in the 
    % current axes
    condition2 = ishold && isempty(findobj(gca, 'Tag', robot.name));
    if condition1 || condition2
        % no robot with this name exists
        h = create_new_robot(robot, opt);
        % save the handle in the robot object and attach it to the robot 
        % as user data.
        robot.handle = h;
        set(h.robot, 'Tag', robot.name);
        set(h.robot, 'UserData', robot);
        robot_handle = h.robot;        
    end

    % Update all robots with the same name. This is very useful when we
    % want to visualize the same robot from different points of view
    for r = robot_handle'
        update_robot( get(r, 'UserData'), q);
    end

    % save the joint angles away in all the graphical robots with the
    % same name
    for r = robot_handle'
        rr = get(r, 'UserData');
        rr.configuration = q;
        set(robot_handle, 'UserData', rr);
    end
end


% h = create_new_robot(robot, opt) uses data from robot object and options
% to create a graphical robot in the current figure.
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
        plane = robot.base.'*DQ.k*robot.base';
        % Since the plane is infinite, the plot function draws the part
        % closest to the origin of the reference frame. We first
        % consider the plane that passes through the origin and is aligned with 
        % the one that supports the base
        base_handle = plot(plane.P,'plane',opt.mag,'color','b');
        % We then translate the 'visible' plane to the base frame
        base_translation = vec3(translation(robot.base));
        plane_vertices = get(base_handle, 'Vertices');
        for i = 1:3
            plane_vertices(:,i) = plane_vertices(:,i) + base_translation(i);
        end        
        set(base_handle, 'Vertices', plane_vertices);
        h.base_handle = base_handle;
    end
     
    % Write the robot name.
    if opt.name        
        b = vec3(translation(robot.base));
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
        h.xt = text(0, 0, opt.wristlabel(1), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
        h.yt = text(0, 0, opt.wristlabel(2), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
        h.zt = text(0, 0, opt.wristlabel(3), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');

    end

    % Display cylinders (revolute each joint, as well as axis centerline.
    for i = 1:robot.links
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
% specified by the joint coordinates q. Graphics are defined by the handle 
% structure robot.handle.
function update_robot(robot, q)
    % Dummy (virtual) joints will be removed in the near future
    n = robot.links-robot.n_dummy;
    
    h = robot.handle;
    mag = h.mag;
    base = vec3(translation(robot.base));
    
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
        t = vec3(translation(robot.fkm(q,j)));
        x(j+1) = t(1);    
        y(j+1) = t(2);
        z(j+1) = t(3);
    end
    
    % Update the coordinates of each frame along the kinematic chain   
    set(h.robot,'xdata', x, 'ydata', y, 'zdata', z);
   
    % display the joints as cylinders
    if isfield(h, 'joint')       
        for j=1:n
            % get coordinate data from the cylinder. The UserData is never
            % updated.
            xyz = get(h.joint(j), 'UserData');
            
            %The joints are located at the beginning of each link
            fkm_j = robot.fkm(q,j-1);
            
            for k = 1:size(xyz,2)
                % 1 + DQ.E*(1/2)*p, where p = xyz(1:3,k);
                cylinder_position = DQ([1;0;0;0;0;0.5*xyz(1:3,k)]);
                xyz(1:3,k) = vec3(translation(fkm_j*cylinder_position));
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
        t = robot.fkm(q);
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


% o = PLOT_OPTIONS(robot, options) returns an options structure
function o = plot_options(robot, optin)
    % process a cell array of options and return a struct

    % define all possible options and their default values
  %  o.erasemode = 'normal';
    o.joints = true;
    o.wrist = true;
    o.loop = false;
  %  o.shadow = false;
    o.wrist = true;
    o.base = true;
    o.wristlabel = 'xyz';
    o.perspective = true;
    o.magscale = 1;
    o.name = true;
    o.cylinder = [0 0 0.7];
    o.workspace = [];

    % build a list of options from all sources
    %   1. the M-file plotbotopt if it exists
    %   2. robot.plotopt
    %   3. command line arguments
%     if exist('plotbotopt', 'file') == 2
%         options = [plotbotopt robot.plotopt optin];
%     else
        options = [robot.plotopt optin];
 %   end

    % parse the options
    [o,args] = tb_optparse(o, options);
    if length(args) > 0
        error(['unknown option: ' args{1}]);
    end

    if isempty(o.workspace)
        %
        % simple heuristic to figure the maximum reach of the robot
        %        
        reach = 0;
        for i=1:robot.links
            reach = reach + abs(robot.a(i)) + abs(robot.d(i));
        end
        o.workspace = [-reach reach -reach reach -reach reach];
        o.mag = reach/10;
    else
        reach = min(abs(o.workspace));
    end
    o.mag = o.magscale * reach/10;
end

%OPTPARSE Standard option parser for Toolbox functions
%
% [OPTOUT,ARGS] = TB_OPTPARSE(OPT, ARGLIST) is a generalized option parser for
% Toolbox functions.  It supports options that have an assigned value, boolean 
% or enumeration types (string or int).
%
% The software pattern is:
%
%       function(a, b, c, varargin)
%       opt.foo = true;
%       opt.bar = false;
%       opt.blah = [];
%       opt.choose = {'this', 'that', 'other'};
%       opt.select = {'#no', '#yes'};
%       opt = tb_optparse(opt, varargin);
%
% Optional arguments to the function behave as follows:
%   'foo'           sets opt.foo <- true
%   'nobar'         sets opt.foo <- false
%   'blah', 3       sets opt.blah <- 3
%   'blah', {x,y}   sets opt.blah <- {x,y}
%   'that'          sets opt.choose <- 'that'
%   'yes'           sets opt.select <- 2 (the second element)
%
% and can be given in any combination.
%
% If neither of 'this', 'that' or 'other' are specified then opt.choose <- 'this'.
% If neither of 'no' or 'yes' are specified then opt.select <- 1.
%
% Note:
% - that the enumerator names must be distinct from the field names.
% - that only one value can be assigned to a field, if multiple values
%    are required they must be converted to a cell array.
%
% The allowable options are specified by the names of the fields in the
% structure opt.  By default if an option is given that is not a field of 
% opt an error is declared.  
%
% Sometimes it is useful to collect the unassigned options and this can be 
% achieved using a second output argument
%           [opt,arglist] = tb_optparse(opt, varargin);
% which is a cell array of all unassigned arguments in the order given in
% varargin.
%
% The return structure is automatically populated with fields: verbose and
% debug.  The following options are automatically parsed:
%   'verbose'           sets opt.verbose <- true
%   'verbose=2'         sets opt.verbose <- 2 (very verbose)
%   'verbose=3'         sets opt.verbose <- 3 (extremeley verbose)
%   'verbose=4'         sets opt.verbose <- 4 (ridiculously verbose)
%   'debug', N          sets opt.debug <- N
%   'setopt', S         sets opt <- S
%   'showopt'           displays opt and arglist

function [opt,others] = tb_optparse(in, argv)

    arglist = {};

    argc = 1;
    opt = in;
    try
        opt.verbose = false;
        opt.debug = 0;
    end

    showopt = false;

    while argc <= length(argv)
        option = argv{argc};
        assigned = false;
        
        if isstr(option)

            switch option
            % look for hardwired options
            case 'verbose'
                opt.verbose = true;
                assigned = true;
            case 'verbose=2'
                opt.verbose = 2;
                assigned = true;
            case 'verbose=3'
                opt.verbose = 3;
                assigned = true;
            case 'verbose=4'
                opt.verbose = 4;
                assigned = true;
            case 'debug'
                opt.debug = argv{argc+1};
                argc = argc+1;
                assigned = true;
            case 'setopt'
                new = argv{argc+1};
                argc = argc+1;
                assigned = true;


                % copy matching field names from new opt struct to current one
                for f=fieldnames(new)'
                    if isfield(opt, f{1})
                        opt = setfield(opt, f{1}, getfield(new, f{1}));
                    end
                end
            case 'showopt'
                showopt = true;
                assigned = true;

            otherwise
                % does the option match a field in the opt structure?
                if isfield(opt, option)
                    val = getfield(opt, option);
                    if islogical(val)
                        % a logical variable can only be set by an option
                        opt = setfield(opt, option, true);
                    else
                        % otherwise grab its value from the next arg
                        opt = setfield(opt, option, argv{argc+1});
                        argc = argc+1;
                    end
                    assigned = true;
                elseif length(option)>2 && strcmp(option(1:2), 'no') && isfield(opt, option(3:end))
                    val = getfield(opt, option(3:end));
                    if islogical(val)
                        % a logical variable can only be set by an option
                        opt = setfield(opt, option(3:end), false);
                        assigned = true;
                    end
                else
                    % the option doesnt match a field name
                    for field=fieldnames(opt)'
                        val = getfield(opt, field{1});
                        if iscell(val)
                            for i=1:length(val)
                                if isempty(val{i})
                                    continue;
                                end
                                if strcmp(option, val{i})
                                    opt = setfield(opt, field{1}, option);
                                    assigned = true;
                                    break;
                                elseif val{i}(1) == '#' && strcmp(option, val{i}(2:end))
                                    opt = setfield(opt, field{1}, i);
                                    assigned = true;
                                    break;
                                end
                            end
                            if assigned
                                break;
                            end
                        end
                    end


                end
            end % switch
        end
        if ~assigned
            % non matching options are collected
            if nargout == 2
                arglist = [arglist argv(argc)];
            else
                if isstr(argv{argc})
                    error(['unknown options: ' argv{argc}]);
                end
            end
        end
        
        argc = argc + 1;
    end % while

    % if enumerator value not assigned, set the default value
    for field=fieldnames(in)'
        if iscell(getfield(in, field{1})) && iscell(getfield(opt, field{1}))
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
                        
    if showopt
        fprintf('Options:\n');
        opt
        arglist
    end

    if nargout == 2
        others = arglist;
    end
end
