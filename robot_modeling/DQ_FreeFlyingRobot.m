% Basic implementation of a free-flying robot. It is plotted as a small
% sphere with a small frame attached on it if its radius is greater than
% zero; otherwise, only the frame is shown.
% 
% Usage: robot = DQ_FreeFlyingRobot()
%
% DQ_FreeFlyingRobot Properties:
%       radius - The default sphere radius is 0.25 m.
%
% DQ_FreeFlyingRobot Methods (Concrete):
%       fkm - Compute the mobile base pose while considering any base offset.
%       create_new_robot (Protected) - Create new robot graphics.
%       pose_jacobian - Compute the mobile base Jacobian while considering the base offset.
%       set_radius - Change the sphere radius.
%       update_robot (Protected) - Move an existing graphical robot to a specified configuration.
%
%       See also DQ_MobileBase, DQ_DifferentialDriveRobot, DQ_HolonomicBase

% (C) Copyright 2011-2023 DQ Robotics Developers
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
%     Bruno Vilhena Adorno - adorno@ieee.org

classdef DQ_FreeFlyingRobot < DQ_Kinematics
    
    properties (Access = protected)
        radius = 0.25; % The default sphere radius is 0.25 m
    end
    
    properties (Access = private)
        shape_template;
        handle;
    end
    
    methods
        function obj = DQ_FreeFlyingRobot()
            % This robot configuration is given by the unit dual quaternion
            % that represents it's pose. Although strictly speaking this is
            % a 6-dimensional manifold, here we're counting the numbers of
            % coefficients in the unit dual quaternion. Recall that a unit
            % dual quaternion has two constraints, therefore even though it
            % has eight coefficients, these two constraints decreases the
            % number of dimensions of the underlying manifold.
            obj.dim_configuration_space = 8;            
        end
       
        
        function pose = fkm(~, x, ~)
        % FKM(q) returns the free-flying robot pose.
        
            if nargin > 3
                error('Invalid number of arguments');
            else
                pose = x;
            end
        end
            
        function dim = get_dim_configuration_space(obj)
            dim = obj.dim_configuration_space;
        end
        
        function plot(robot, q)
            if isempty(robot.handle)
                robot.handle = robot.create_new_robot();
            end
            
            update_robot(robot,q);
        end
        
        function J = pose_jacobian(~,x, ~)
        % POSE_JACOBIAN(x) returns, given the unit dual quaternion x that
        % represents the free-flying robot configuration, the pose Jacobian
        % J that satisfies x_dot = J*twist, where x_dot is the time
        % derivative of the unit dual quaternion that represents the
        % free-flying robot pose, and twist = vec8(csi), where csi
        % satisfies the dual quaternion propagation equation 
        % xdot = (1/2)*csi*x.
            if nargin > 3
                error('Invalid number of arguments');
            else
                % calculates the mobile robot pose Jacobian. If an index is
                % passed as an argument, just ignore it because there is
                % only one element in the chain. However, we have to tackle
                % this case to ensure compatibility with other
                % DQ_Kinematics objects.
                 J = haminus8(x/2);
            end
        end
        
        function set_radius(obj,radius)
            % Set the sphere radius, in meters. This function must be
            % called before the first plot to take effect in the
            % visualization.
            obj.radius = radius;
        end
    end
    
    methods (Access = protected)
        function h = create_new_robot(robot, opt)
            % h = CREATE_NEW_ROBOT(robot, opt) uses data from robot object and
            % options to create a graphical robot. It returns a structure of handles
            % to graphical objects.            
            % The handle is composed of the following fields:
            % h.scale	robot scale
            % h.robot	the geometrical primitive that represents the robot
            % h.x       the line segment that represents the base frame x-axis
            % h.y       the line segment that represents the base frame y-axis
            % h.z       the line segment that represents the base frame z-axis
            % h.xt      text for the base x-axis
            % h.yt      text for the base y-axis
            % h.zt      text for the base z-axis
            
         %   if nargin > 2
              %  h.scale = opt.scale;
                 % create frame
                 %   if opt.frame
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
                    h.xt = text(0, 0, 'x', 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
                    h.yt = text(0, 0, 'y', 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
                    h.zt = text(0, 0, 'z', 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
            %    end
           % else
                h.scale = 2;
          %  end
            
            if ~ishold
                % if current figure has hold on, then draw robot here
                % otherwise, create a new figure
                axis tight;
            end
            xlabel('X')
            ylabel('Y')
            zlabel('Z')
            set(gca, 'SortMethod', 'depth');
            grid on
                        
            
            % Store all surface points needed to draw a sphere later on.
            [robot.shape_template.X,robot.shape_template.Y,...
                robot.shape_template.Z] = sphere(6);
            
            % Draw the actual sphere to initialize the plot system.
            h.robot_shape = surface(robot.shape_template.X, ...
                robot.shape_template.Y,robot.shape_template.Z);
        end        
       
        function update_robot(robot, q)
            % Get the handle to the graphical robot. Since each kinematic
            % robot stores just one graphical handle, if we want to plot
            % the same robot in different views, we must declare different
            % robots with the same name. Otherwise, if just one robot is
            % declared, but plotted in different windows/views, the
            % kinematic robot will store the handle of the last view only.
            h = robot.handle;
          %  mag = h.scale*robot.radius;
          
            mag = robot.radius;
            % base = vec3(translation(robot.base));
           
            % Get the origin of the frame. Recall that, for a free-flying
            % robot, its configuration q equals its pose.
            p = vec3(translation(q)); 
            x = p(1);
            y = p(2);
            z = p(3);

            % Update the free-flying robot's coordinates
            set(h.robot_shape, 'XData',robot.shape_template.X * mag + x);
            set(h.robot_shape, 'YData',robot.shape_template.Y * mag + y);
            set(h.robot_shape, 'ZData',robot.shape_template.Z * mag + z);

            % display the wrist axes and labels   
            % compute the wrist axes, based on final link transformation plus the
            % tool transformation.
            if isfield(h, 'x')
                % recall that the free-flying robot configuration is the
                % robot pose.
                pose = q;
                 

                % The following transformations use the Hamilton operators to
                % improve performance as Matlab is optimized for matrix operations.

                % Recall that, given a rotation t.P, the point transformation of any
                % point p is given by t.P*p*t.P'. In terms of Hamilton operators,
                % the transformation is given by 
                % vec4(t.P*p*t.P') = hamiplus4(t.P)*haminus4(t.P')*vec4(p).

                % H(1,2:4) is always 0.
                H = hamiplus4(pose.P)*haminus4(pose.P');

                % We can simplify the calculations as follows
                xv = p + H(2:4,2)*mag*h.scale; % p = [0; mag; 0; 0]
                yv = p + H(2:4,3)*mag*h.scale; % p = [0; 0; mag; 0]
                zv = p + H(2:4,4)*mag*h.scale; % p = [0; 0; 0; mag]        

                % update the wrist axes       
                set(h.x,'xdata',[p(1) xv(1)], 'ydata', [p(2) xv(2)], ...
                    'zdata', [p(3) xv(3)]);
                set(h.y,'xdata',[p(1) yv(1)], 'ydata', [p(2) yv(2)], ...
                     'zdata', [p(3) yv(3)]);
                set(h.z,'xdata',[p(1) zv(1)], 'ydata', [p(2) zv(2)], ...
                     'zdata', [p(3) zv(3)]);

                % update the axes' name positions 
                set(h.xt, 'Position', xv);
                set(h.yt, 'Position', yv);
                set(h.zt, 'Position', zv);
            end
        end         
    end
end

