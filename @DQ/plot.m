% h = plot(dq,OPTIONS) plots the dual quaternion dq and returns the handle to
% the plot.
%
% The following options are available:
%   'scale'
%   'erase'
%   'line'
%   'plane'
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
% plot(dq,'line',length) will plot the Plucker line represented by the unit
% dual quaternion dq. Since a Plucker line is infinite, length is used to
% determine the size of the "visible" line to be plotted.

% plot(dq,'plane',size) will plot the plane represented by the unit dual
% quaternion dq = n + DQ.E*d, where 'n' is the pure unit norm quaternion 
% representing the vector normal to the plane and 'd' is the signed distance of
% the plane with respect to the origin of the reference frame. Since a plane is 
% infinite, size is used to determine the size of the visible plane.

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

if optargin > 0
    % All parameters in the variable-length list have the form <type>
    % folowed by the corresponding type value. Therefore, length(varargin)
    % is always even.
    for j = 1:2:optargin
        
        % Convert the cell to a string in order to use the SWITCH command.
        my_string = char(varargin(j));
        
        switch lower(my_string)
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
            otherwise
                warning('Unknown plot parameter.')
        end
    end
end

% The variable parameter list has been unwrapped. Now we draw specific
% primitives
switch primitive_type
    case 'frame'
        % TODO: Document the line.
        if erase
            % Retrieve the struct inside the cell
            handle = handle_cell{1};
            for i = 1:3
                delete(handle.handle_axis{i});
                delete(handle.handle_text{i});
            end
        end
        
        % create unit vectors and rotate them by the quaternion part of dq.
        t1 = 1+DQ.E*[0, scale*0.5, 0, 0];
        t2 = 1+DQ.E*[0, 0, scale*0.5, 0];
        t3 = 1+DQ.E*[0, 0, 0, scale*0.5];
        
        %Do a vector rotation
        x = dq.P*t1*dq.P';
        y = dq.P*t2*dq.P';
        z = dq.P*t3*dq.P';
        
        old_ishold = ishold;
        
        dq_t = translation(dq);
        x_t  = translation(x);
        y_t  = translation(y);
        z_t  = translation(z);
        
        handle.handle_axis{1} = plot3([0; x_t.q(2)] + dq_t.q(2), ...
            [0; x_t.q(3)] + dq_t.q(3), [0; x_t.q(4)] + dq_t.q(4), ...
            'r','Linewidth',1);
        
        if(~ishold)
            hold on;
        end
        
        handle.handle_text{1} = text(dq_t.q(2)+x_t.q(2), dq_t.q(3)+x_t.q(3), dq_t.q(4)+x_t.q(4), 'x');
        set(handle.handle_text{1} , 'Color', 'k');
        
        handle.handle_axis{2} = plot3([0;y_t.q(2)]+dq_t.q(2), [0; y_t.q(3)]+dq_t.q(3), [0; y_t.q(4)]+dq_t.q(4), 'g','Linewidth',1);
        handle.handle_text{2} = text(dq_t.q(2)+y_t.q(2), dq_t.q(3)+y_t.q(3), dq_t.q(4)+y_t.q(4), 'y');
        set(handle.handle_text{2}, 'Color', 'k');
        
        handle.handle_axis{3} = plot3([0;z_t.q(2)]+dq_t.q(2), [0; z_t.q(3)]+dq_t.q(3), [0; z_t.q(4)]+dq_t.q(4), 'b','Linewidth',1);
        handle.handle_text{3}  = text(dq_t.q(2)+z_t.q(2), dq_t.q(3)+z_t.q(3), dq_t.q(4)+z_t.q(4), 'z');
        set(handle.handle_text{3}, 'Color', 'k');
        if(~old_ishold)
            hold off;
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
            set(handle_cell{1}, 'XData', arg{1}','YData', arg{2}','ZData', arg{3}');
        else
            handle = plot3(arg{:});
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
            sqr_norm_v = norm(v)*norm(v);
            v = v*inv(sqr_norm_v)*(plane_length/2);
        else
            v = DQ.j; %DQ.k would be another obvious option
        end
        
        % u,v,n are perpedicular
        u = cross(n,v);
        
        p = n*d; % point on the plane
        
        
        p1 = vec3(p + u);
        p2 = vec3(p + v);
        p3 = vec3(p - u);
        p4 = vec3(p - v);
        
        arg = mat2cell([p1';p2';p3';p4'],4,[1,1,1]);
        
        handle = patch(arg{:},'b');
        
        % TODO: implement erase option
        
        % TODO: implement color option
end
end