% h = plot(dq,OPTIONS) plots the dual quaternion dq and returns the handle to
% the plot.
%
% The following options are available:
%   
%   'scale' - Define the scale for the drawing frames.
%   'erase' - Whenever a primitive is moved, 'erase' prevents leaving a trail.
%   'name'  - Define frame names (Latex commands are allowed).
%   'line'  - Draw a line using dual quaternion representation.
%   'plane' - Draw a plane using dual quaternion representation.
%   'color' - Define line and plane colors.   
%   'noaxisname' - Do not plot the axes' names.
%
% Note: All options require an option attribute.
%
% Examples:
% plot(dq,'scale',X) will plot the unit dual quaternion dq with the axes scaled
% by a factor of X.
%
% plot(dq,'erase', h) will plot the unit dual quaternion dq, but erasing
% previous values of dq. The parameter h is the handle of the plot. This is
% useful for plotting frames in motion (if erase is not used, it will leave a
% trail).
%
% plot(dq, 'name', '$F_0$') will plot a frame with name F_0. Since '$' is
% used, F_0 is interpreted as a latex math formula.
%
% plot(dq,'line',length) will plot the Plucker line represented by the unit
% dual quaternion dq. Since a Plucker line is infinite, length is used to
% determine the size of the "visible" line to be plotted.
%
% plot(dq,'plane',size) will plot the plane represented by the unit dual
% quaternion dq = n + DQ.E*d, where 'n' is the pure unit norm quaternion 
% representing the vector normal to the plane and 'd' is the signed distance of
% the plane with respect to the origin of the reference frame. Since a plane is 
% infinite, size is used to determine the diagonal of the visible plane.
%
% plot(dq,'plane', size, 'color', 'r') will plot a red plane.


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
%     Part of this file was derived from Peter Corke's Robotics Toolbox
%     (http://www.petercorke.com).

function handle = plot(dq,varargin)

if norm(dq) ~= 1   
    error('Only unit dual quaternions can be plotted');
end

optargin = length(varargin);
erase = 0; % Only erase the primitive only if there is a corresponding argument.
primitive_type = 'frame'; % The default primitive is a coordinate system.
scale = 1; % Variable for scaling coordinate systems.
noaxisname = false; % Default behavior is to print the axes names

if optargin > 0
    % All parameters in the variable-length list have the form <type>
    % folowed by the corresponding type value. Therefore, length(varargin)
    % is always even.
    for j = 1:2:optargin
        
        % Convert the cell to a string in order to use the SWITCH command.
        option_name = char(varargin(j));
        
        switch lower(option_name)
            case 'erase'
                erase = 1;
                handle_cell = varargin(j+1);
            case 'scale'
                scale = cell2mat(varargin(j+1));
            case 'line'
                primitive_type = 'line';
                line_length = cell2mat(varargin(j+1));
            case 'plane'
                primitive_type = 'plane';
                plane_length = cell2mat(varargin(j+1));
            case 'name'
                frame_name = char(varargin(j+1));
            case 'color'
                primitive_color = char(varargin(j+1));
            case 'noaxisname'
                noaxisname = true;
            otherwise
                warning('Unknown plot parameter: %s', option_name);
        end
    end
end

% The variable parameter list has been unwrapped. Now we draw specific
% primitives
switch primitive_type
    case 'frame'

        if erase
            % Retrieve the struct inside the cell
            handle = handle_cell{1};
            for i = 1:3
                % TODO: Maybe a better option is to move the axes and the
                % text instead of deleting them.
                delete(handle.handle_axis{i});
                if noaxisname == false
                    delete(handle.handle_text{i});    
                end
            end
        end
        
        % Create vectors along the x-, y-, and z-axes.
        t1 = scale*DQ.i;
        t2 = scale*DQ.j;
        t3 = scale*DQ.k;
        
        % Do a point rotation
        x = vec3(dq.P*t1*dq.P');
        y = vec3(dq.P*t2*dq.P');
        z = vec3(dq.P*t3*dq.P');        
        
        old_ishold = ishold;
        
        dq_t = vec3(translation(dq));
        
        % Plot the x-axis.
        % Recall that [0;x(1)] + dq_t(1) = [dq_t(1); x(1) + dq_t(1)]
        handle.handle_axis{1} = plot3([0;x(1)] + dq_t(1), ...
            [0;x(2)] + dq_t(2), [0;x(3)] + dq_t(3), 'r','Linewidth',1);
        
        if(~ishold)
            hold on;
        end
        
        
        % Plot the y-axis
        handle.handle_axis{2} = plot3([0;y(1)] + dq_t(1), [0;y(2)] + dq_t(2),...
            [0; y(3)] + dq_t(3), 'g','Linewidth',1);
        
        % Plot the z-axis
        handle.handle_axis{3} = plot3([0;z(1)] + dq_t(1), [0;z(2)] + dq_t(2),...
            [0,z(3)] + dq_t(3), 'b','Linewidth',1);
        
        % Plot the axes texts
        
        if noaxisname == false
            handle.handle_text{1} = text(dq_t(1) + x(1), dq_t(2) + x(2),...
                dq_t(3) + x(3), 'x');
            set(handle.handle_text{1} , 'Color', 'k');
            handle.handle_text{2} = text(y(1) + dq_t(1), y(2) + dq_t(2),...
                y(3) + dq_t(3), 'y');
            set(handle.handle_text{2}, 'Color', 'k');
            handle.handle_text{3}  = text(z(1) + dq_t(1), z(2) + dq_t(2),...
                z(3) + dq_t(3), 'z');
            set(handle.handle_text{3}, 'Color', 'k');
        end
        
        
        if(~old_ishold)
            hold off;
        end
        
        % Plot the frame name in case the option 'name' is used
        if exist('frame_name','var')
            % Verify if the handle_name field exists, that is, the frame
            % already has a name
            if isfield(handle, 'handle_name')
                set(handle.handle_name,'Position',dq_t'+ scale*0.05);
            % In case the frame does not have a name, create a new one
            else
                handle.handle_name = text(dq_t(1) + scale*0.05, dq_t(2) + ...
                scale*0.05, dq_t(3) + scale*0.05, frame_name,'Interpreter',...
                'latex');
            end
        end
        
    case 'line'
        if (dq.Re ~= 0) || (norm(dq) ~= 1)
            error(['The dual quaternion does not represent a Plucker line.'...
                'It must be imaginary and have unit norm']);
        end
        
        ld = vec3(dq.P); % line direction
        lm = vec3(dq.D); % line moment
        
        % Find the skew-symmetric matrix of ld.
        S = [0    , -ld(3),  ld(2) ;
            ld(3) , 0     , -ld(1) ;
            -ld(2), ld(1) ,     0 ];
        
        p = -pinv(S)*lm; % Find point on the line closest to the reference frame
        p1 = p + ld*line_length/2;
        p2 = p - ld*line_length/2;
        
        arg = mat2cell([p1';p2'],2,[1,1,1]);
        
        if erase
            handle = handle_cell{1};
            set(handle_cell{1}, 'XData', arg{1}','YData', arg{2}','ZData',...
                arg{3}');
        else
            if exist('primitive_color', 'var');
                handle = plot3(arg{:}, primitive_color);
            else % Use matlab default colors
                handle = plot3(arg{:});
            end
        end
        
    case 'plane'
        % A plane is given by plane = n + DQ.E*d, where n is a pure unit
        % quaternion and d is a real number.
        if (dq.P.Re ~= 0) || (norm(dq) ~= 1) || (dq.D.Im ~= 0)
            error(['The dual quaternion does not represent a Plane.'...
                'It must have unit norm, the primary part must be pure and '...
                'the dual part must have null imaginary component.']);
        end
        
        % This implementation uses only dual quaternion algebra, mainly for 
        % pedagogical purposes, but it can be simplified if computional 
        % efficiency is needed.
        n = dq.P;
        d = dq.D;    
       
        % Now we obtain a vector orthogonal to n.
        if n ~= DQ.i
            v = cross(DQ.i,n);            
            v = (v/norm(v))*(plane_length/2);            
        else
            v = DQ.j*(plane_length/2); %DQ.k would be another obvious option
        end
        
        % u,v,n are perpedicular
        u = cross(n,v);
        
        p = n*d; % point on the plane
        
        p1 = vec3(p + u);
        p2 = vec3(p + v);
        p3 = vec3(p - u);
        p4 = vec3(p - v);
        
        plane_coordinates = [p1';p2';p3';p4'];
        
        if erase
            handle = handle_cell{1};
            set(handle_cell{1},'Vertices',plane_coordinates);
        else
            arg = mat2cell(plane_coordinates,4,[1,1,1]);
            if exist('primitive_color', 'var')
                handle = patch(arg{:}, primitive_color);
            else % Use the default color (blue)
                handle = patch(arg{:}, 'b');
            end                
        end
        
        % TODO: implement erase option
        
        % TODO: implement color option
end
end