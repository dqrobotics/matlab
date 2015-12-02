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
%     Part of this file was derived from Peter Corke's Robotic Toolbox (http://www.petercorke.com)

function plot(obj,joint_angles,varargin)
    
%     robot = obj.robot_RT;
    has_options = 0;
    if nargin < 2
        error('Usage: plot(robot,theta,[options])');
    elseif nargin >= 3
        has_options = 1;
    end
    
    if size(joint_angles,2) == 1
        joint_angles = joint_angles';
    end
    
    % If there are dummy joints, we must plot them
    if obj.n_dummy
        theta = zeros(1,length(obj.theta));
        counter = 0;
        for i = 1:length(theta)
            if obj.dummy(i) == 1
                theta(i) = obj.theta(i);
                counter = counter + 1;
            else
                theta(i)=joint_angles(i-counter)+obj.theta(i);
            end
            
        end
    else        
        theta = joint_angles;
    end
    
    
    if has_options
        rt_plot(obj,joint_angles, varargin{:});
    else
        rt_plot(obj,joint_angles);
    end
end


%SerialLink.ploT Graphical display and animation
%
% R.plot(Q, options) displays a graphical animation of a robot based on 
% the kinematic model.  A stick figure polyline joins the origins of
% the link coordinate frames. The robot is displayed at the joint angle Q, or 
% if a matrix it is animated as the robot moves along the trajectory.
%
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
% - shadow on the floor
% - XYZ wrist axes and labels
% - joint cylinders and axes
% which are controlled by options.
%
% The size of the annotations is determined using a simple heuristic from 
% the workspace dimensions.  This dimension can be changed by setting the 
% multiplicative scale factor using the 'mag' option.
%
% Options::
%  'workspace', W          size of robot 3D workspace, W = [xmn, xmx ymn ymx zmn zmx]
%  'delay', d              delay betwen frames for animation (s)
%  'cylinder', C           color for joint cylinders, C=[r g b]
%  'mag', scale            annotation scale factor
%  'perspective'|'ortho'   type of camera view
%  'raise'|'noraise'       controls autoraise of current figure on plot
%  'render'|'norender'     controls shaded rendering after drawing
%  'loop'|'noloop'         controls endless loop mode
%  'base'|'nobase'         controls display of base 'pedestal'
%  'wrist'|'nowrist'       controls display of wrist
%  'shadow'|'noshadow'     controls display of shadow
%  'name'|'noname'         display the robot's name 
%  'xyz'|'noa'             wrist axis label
%  'jaxes'|'nojaxes'       control display of joint axes
%  'joints'|'nojoints'     controls display of joints
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
%   h.shadow    the robot's shadow
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



% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function retval = rt_plot(robot, tg, varargin)
    % opts = PLOT(robot, options)
    %  just convert options list to an options struct
    if (nargin == 2) && iscell(tg)
        retval = plot_options(robot, varargin{:});
        return;
    end
    
    % process options
    if (nargin > 2) && isstruct(varargin{1})
        % options is a struct
        opt = varargin{1};
    else
        % options is a list of options
        opt = plot_options(robot, varargin);
    end


    n = robot.links-robot.n_dummy;

    if length(tg) ~= n
        error('Insufficient number of joints')
    end

    % get handle of any existing robot of same name
    rh = findobj('Tag', robot.name);

    if isempty(rh) || isempty( get(gcf, 'Children'))
        % no robot of this name exists

        % create one
        ax = newplot();
        h = create_new_robot(robot, opt);

        % save the handles in the passed robot object, and
        % attach it to the robot as user data.
        robot.handle = h;
        set(h.robot, 'Tag', robot.name);
        set(h.robot, 'UserData', robot);

        rh = h.robot;
    end

    if ishold && isempty( findobj(gca, 'Tag', robot.name))
        % if hold is on and no robot of this name in current axes
        h = create_new_robot(robot, opt);
        % save the handles in the passed robot object, and
        % attach it to the robot as user data.
        robot.handle = h;
        set(h.robot, 'Tag', robot.name);
        set(h.robot, 'UserData', robot);

        rh = h.robot;
    end
    
    if opt.raise
        figure(gcf);
    end



    for r=rh'
        animate( get(r, 'UserData'), tg, opt);

    end



    % save the last joint angles away in the graphical robot
    for r=rh'
        rr = get(r, 'UserData');
        rr.q = tg(end,:);
        set(r, 'UserData', rr);
    end

    if nargout > 0
        retval = robot;
    end
    
end
%PLOT_OPTIONS
%
%   o = PLOT_OPTIONS(robot, options)
%
% Returns an options structure

function o = plot_options(robot, optin)
    % process a cell array of options and return a struct

    % define all possible options and their default values
    o.erasemode = 'normal';
    o.joints = true;
    o.wrist = true;
    o.loop = false;
    o.shadow = true;
    o.wrist = true;
    o.jaxes = true;
    o.base = true;
    o.wristlabel = 'xyz';
    o.perspective = true;
    o.magscale = 1;
    o.name = true;
    o.delay = 0.1;
    o.raise = false;
    o.cylinder = [0 0 0.7];
    o.workspace = [];

    % build a list of options from all sources
    %   1. the M-file plotbotopt if it exists
    %   2. robot.plotopt
    %   3. command line arguments
    if exist('plotbotopt', 'file') == 2
        options = [plotbotopt robot.plotopt optin];
    else
        options = [robot.plotopt optin];
    end

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

%CREATE_NEW_ROBOT
% 
%   h = CREATE_NEW_ROBOT(robot, opt)
%
% Using data from robot object and options create a graphical robot in
% the current figure.
%
% Returns a structure of handles to graphical objects.
%
% If current figure is empty, draw robot in it
% If current figure has hold on, add robot to it
% Otherwise, create new figure and draw robot in it.
%   

% h.mag
% h.zmin
% h.robot   the line segment that is the robot
% h.shadow  the robot's shadow
% h.x       the line segment that is T6 x-axis
% h.y       the line segment that is T6 x-axis
% h.z       the line segment that is T6 x-axis
% h.xt      text for T6 x-axis
% h.yt      text for T6 y-axis
% h.zt      text for T6 z-axis
% h.joint(i)        the joint i cylinder or box
% h.jointaxis(i)    the line segment that is joint i axis
% h.jointlabel(i)   text for joint i label

function h = create_new_robot(robot, opt)
    h.mag = opt.mag;

    %
    % setup an axis in which to animate the robot
    %
    % handles not provided, create graphics
    %disp('in creat_new_robot')
    if ~ishold
        % if current figure has hold on, then draw robot here
        % otherwise, create a new figure
        axis(opt.workspace);
    end
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    set(gca, 'drawmode', 'fast');
    grid on


    zlim = get(gca, 'ZLim');
    h.zmin = zlim(1);

    if opt.base
        b = translation(robot.base);
        line('xdata', [b.q(2);b.q(2)], ...
            'ydata', [b.q(3);b.q(3)], ...
            'zdata', [h.zmin;b.q(4)], ...
            'LineWidth', 4, ...
            'color', 'red');
    end
    
    if opt.name
        b = translation(robot.base);
        text(b.q(2), b.q(3)-opt.mag, [' ' robot.name], 'FontAngle', 'italic', 'FontWeight', 'bold')
    end
    % create a line which we will
    % subsequently modify.  Set erase mode to xor for fast
    % update
    %
    h.robot = line(robot.lineopt{:});
    
    if opt.shadow
        h.shadow = line(robot.shadowopt{:}, ...
            'Erasemode', opt.erasemode);
    end

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

    %
    % display cylinders (revolute) or boxes (pristmatic) at
    % each joint, as well as axis centerline.
    %
    
    for i=1:robot.links
        
        if opt.joints

            % cylinder or box to represent the joint
            %if L(i).sigma == 0
                N = 8;
            %else
             %   N = 4;
            %end
            % define the vertices of the cylinder
            [xc,yc,zc] = cylinder(opt.mag/4, N);
            zc(zc==0) = -opt.mag/2;
            zc(zc==1) = opt.mag/2;

            % create vertex color data
            cdata = zeros(size(xc));
            for j=1:3
                cdata(:,:,j) = opt.cylinder(j);
            end
            % render the surface
            h.joint(i) = surface(xc,yc,zc,cdata);
            
            % set the surfaces to be smoothed and translucent
            set(h.joint(i), 'FaceColor', 'interp');
            set(h.joint(i), 'EdgeColor', 'none');
            set(h.joint(i), 'FaceAlpha', 0.7);

            % build a matrix of coordinates so we
            % can transform the cylinder in animate()
            % and hang it off the cylinder
            xyz = [xc(:)'; yc(:)'; zc(:)'; ones(1,2*N+2)]; 
            set(h.joint(i), 'UserData', xyz);
        end

        if opt.jaxes
            % add a dashed line along the axis
            h.jointaxis(i) = line('xdata', [0;0], ...
                'ydata', [0;0], ...
                'zdata', [0;0], ...
                'color', 'blue', ...
                'linestyle', ':');
            h.jointlabel(i) = text(0, 0, 0, num2str(i), 'HorizontalAlignment', 'Center');
        end
    end
end
%ANIMATE   move an existing graphical robot
%
%   animate(robot, q)
%
% Move the graphical robot to the pose specified by the joint coordinates q.
% Graphics are defined by the handle structure robot.handle.

function animate(robot, q, opt)

    n = robot.links-robot.n_dummy;
    h = robot.handle;
    
    mag = h.mag;

    b = translation(robot.base);
    x = b.q(2);
    y = b.q(3);
    z = b.q(4);

    xs = b.q(2);
    ys = b.q(3);
    zs = h.zmin;

    % compute the link transforms, and record the origin of each frame
    % for the animation.    
    for j=1:n
    
        t = translation(robot.fkm(q,j));

        x = [x; t.q(2)];    
        y = [y; t.q(3)];
        z = [z; t.q(4)];
        xs = [xs; t.q(2)];
        ys = [ys; t.q(3)];
        zs = [zs; h.zmin];
    end

    t = robot.fkm(q);

    %
    % draw the robot stick figure and the shadow
    %
    set(h.robot,'xdata', x, 'ydata', y, 'zdata', z);
    if isfield(h, 'shadow')
        set(h.shadow,'xdata', xs, 'ydata', ys, 'zdata', zs);
    end
    

    %
    % display the joints as cylinders with rotation axes
    %
      if isfield(h, 'joint')
       %  xyz_line = [0 0; 0 0; -2*mag 2*mag; 1 1];
 
         for j=1:n
             
           %  if robot.dummy(j)==0
             % get coordinate data from the cylinder
            xyz = get(h.joint(j), 'UserData');
            fkm_j = robot.fkm(q,j);
            
            for k = 1:numcols(xyz)
                temp = fkm_j*DQ([1;0;0;0;0;0.5*xyz(1:3,k)]);  
                temp = temp.T;
                xyz(1:3,k)=temp.q(6:8)*2;
            end
          
            ncols = numcols(xyz)/2;
            xc = reshape(xyz(1,:), 2, ncols);
            yc = reshape(xyz(2,:), 2, ncols);
            zc = reshape(xyz(3,:), 2, ncols);

            set(h.joint(j), 'Xdata', xc, 'Ydata', yc, ...
                'Zdata', zc);
           %  end

        end
    end

    %
    % display the wrist axes and labels
    %
    if isfield(h, 'x')
        %
        % compute the wrist axes, based on final link transformation
        % plus the tool transformation.
        %
        
        t1 = translation(t);
        
        xv = translation(t*DQ([1 0 0 0 0 mag 0 0]));
        yv = translation(t*DQ([1 0 0 0 0 0 mag 0]));
        zv = translation(t*DQ([1 0 0 0 0 0 0 mag]));

        %
        % update the line segments, wrist axis and links
        %
        set(h.x,'xdata',[t1.q(2) xv.q(2)], 'ydata', [t1.q(3) xv.q(3)], ...
            'zdata', [t1.q(4) xv.q(4)]);
        set(h.y,'xdata',[t1.q(2) yv.q(2)], 'ydata', [t1.q(3) yv.q(3)], ...
             'zdata', [t1.q(4) yv.q(4)]);
        set(h.z,'xdata',[t1.q(2) zv.q(2)], 'ydata', [t1.q(3) zv.q(3)], ...
             'zdata', [t1.q(4) zv.q(4)]);
        set(h.xt, 'Position', xv.q(2:4));
        set(h.yt, 'Position', yv.q(2:4));
        set(h.zt, 'Position', zv.q(2:4));
    end
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

%
% NUMCOLS(m)
%
%	Return the number of columns in the matrix m
%
function c = numcols(m)
	[x,c] = size(m);
end

