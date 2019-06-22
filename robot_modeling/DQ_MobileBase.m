% Abstract class that defines an interface for all mobile robots.
%
% DQ_MobileBase Properties:
%       handle - Graphics handle.
%       dim_configuration_space - Dimension of the configuration space.
%       frame_displacement - Rigid transformation for the base frame.
% 
% DQ_MobileBase Methods (Abstract):
%       create_new_robot (Protected) - Create new robot graphics.
%       raw_fkm - Return the pose given the configuration [x,y,phi]' without considering the base frame displacement. 
%       raw_pose_jacobian - Return the pose Jacobian without considering the base frame displacement.
%       update_robot (Protected) - Move an existing graphical robot to a specified configuration.
%
% DQ_MobileBase Methods (Concrete):
%       get_dim_configuration_space - Return the dimension of the configuration space.
%       plot - Plot the robot.
%       set_frame_displacement - Set the rigid transformation for the base frame.
%
% See also DQ_Kinematics, DQ_HolonomicBase.

% (C) Copyright 2011-2019 DQ Robotics Developers
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
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br

classdef (Abstract) DQ_MobileBase < DQ_Kinematics
    properties      
        plotopt;
    end
    
    properties (Access = protected)
        handle;
        dim_configuration_space;
        frame_displacement;
    end
    
    methods
        function obj = DQ_MobileBase()
            % At the begining, there is no frame displacement (e.g., a 
            % displacement related to the mobile base height).
            obj.frame_displacement = DQ(1);
        end
        
        function set_frame_displacement(obj,x)
            obj.frame_displacement = x;
        end
        
        function ret = get_dim_configuration_space(obj)
            ret = obj.dim_configuration_space;
        end
                
        function plot(robot,q,varargin)
        % PLOT(robot,q,options) plots the robot of type DQ_MobileBase. 
        % q is the vector of joint configurations
        % options is an optional argument that has variable size and accept any
        % number of the following parameters:
        %
        %  'cylinder', C           color for mobile base color, C=[r g b]
        %  'scale', scale          annotation scale factor
        %  'frame'|'noframe'       controls display of base frame
        %  'name'|'noname'         display the robot's name 
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
        % The configuration of the robot as displayed is stored in the
        % DQ_MobileBase object and can be accessed by the read only object
        % property 'q'.
        %
        % 5) Graphical annotations and options:
        %
        % The robot is displayed according to the concrete method
        % create_new_robot() in each subclasses.
        %
        % See also DQ/plot, DQ_kinematics/plot
            
            has_options = 0;
            if nargin < 2
                fprintf(['\nUsage: plot(robot, q,[options])'...
                    '\ntype ''help DQ_MobileBase/plot'' for more '...
                    'information\n']);
                return;
            elseif nargin >= 3
                has_options = 1;
            end

            if ~isvector(q)
                error(['The first argument must be the vector of '...
                       'configurations.']);
            end

            % The joint configuration vector must be a row vector
            if ~isrow(q)
                q = q';
            end

            if has_options
                mobile_base_plot(robot,q, varargin{:});
            else
                mobile_base_plot(robot,q);
            end
        end
	end

    
    methods (Abstract)
         % RAW_POSE_JACOBIAN(obj,q) returns the Jacobian matrix that satisfies 
         % x_dot = J * q, where x_dot is the time derivative of x = FKM(q). It
         % does not take into account the base frame displacement
         J = raw_pose_jacobian(obj, q);
         
       
         % RAW_FKM(q) returns the pose of a mobile base given the 
         % configuration q = [x,y,phi]'. It does not consider the base frame 
         % displacement         
         pose = raw_fkm(obj,q);
    end
    
    methods (Abstract, Access = protected)
        % Each concrete class must implement its own mechanisms for drawing
        % the mobile bases. Therefore, they all have the following methods with
        % their corresponding behaviors:
        
        % h = CREATE_NEW_ROBOT(robot, opt) uses data from robot object and 
        % options to create a graphical robot. It returns a structure of handles
        % to graphical objects.
        
        % It must implement the following behaviors
        %
        % If current figure is empty, draw robot in it
        % If current figure has hold on, add robot to it
        % Otherwise, create new figure and draw robot in it.
        %   
        % The handle is composed of the following fields:
        % h.scale	robot scale
        % h.robot	the geometrical primitive that represents the robot
        % h.x       the line segment that represents the base frame x-axis
        % h.y       the line segment that represents the base frame y-axis
        % h.z       the line segment that represents the base frame z-axis
        h = create_new_robot(robot, options);
        
        % UPDATE_ROBOT(robot, q) moves an existing graphical robot to the 
        % configuration specified by q. The parameter 'robot' is a DQ_MobileBase
        % object, and graphics are defined by the handle structure robot.handle, 
        % which stores the 'graphical robot' as robot.handle.robot.
        update_robot(rr, q);
    end
end

% MOBILE_BASE_PLOT(robot, q, varargin) creates a new robot, if it does
% not exist, otherwise it updates all robots with the same name.
% 
% In case a robot is created, the function CREATE_NEW_ROBOT() provides a
% handle 'h' to the graphical robot. Then, for the h.robot object we 
% additionally: 
%   - save this new kinematic robot object as its UserData
%   - tag it with the name field from the robot object
%
%  This enables us to find all robots with a given name, in all figures,
%  and update them.
function mobile_base_plot(robot, q, varargin)


    % process options
    if (nargin > 2) && isstruct(varargin{1})
        % options is a struct. 
        opt = varargin{1};
    else
        % options is a list of options; we need to transform it to a struct
        opt = plot_options(robot, varargin);
    end

    if length(q) ~= 3
        error('The configuration vector must have three parameters [x;y;phi]');
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

% o = plot_options(robot, options) returns an options structure. 
% 'robot' is the kinematic robot and 'options' is an array cell with the plot
% options.
function o = plot_options(robot, optin)
    % process a cell array of options and return a struct   
    % define all possible options and their default values   
    o.scale = 1; % Scale the drawing
    o.name = true; % Write the robot name
    o.cylinder = [0 0 0.7]; % The mobile base color
    o.frame = true; % Draw the base frame
    
    options = [robot.plotopt optin];
    
    % parse the options
    if ~isempty(options)
        [o,args] = parse_options(o, options);        
        if ~isempty(args)
            error(['Unknown options: ', args{:}]);
        end
    end
end

% [opt, others] = PARSE_OPTIONS(default, options) parses the cell array inside
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

% TODO: Since this function is exactly the same for all classes, maybe we
% should declare it in another place and just use it without having to copy
% it.
    arglist = {};

    argc = 1;
    opt = default;

    while argc <= length(options)
        current_option = options{argc};
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
    others = arglist;    
end


