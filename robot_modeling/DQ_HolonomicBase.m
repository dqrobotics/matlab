% Basic implementation of a holonomic base.
% 
% Usage: robot = DQ_MobileBase()
%
% DQ_HolonomicBase Properties:
%       base_diameter - The default base diameter is 0.5 m.
%
% DQ_HolonomicBase Methods (Concrete):
%       fkm - Compute the mobile base pose while considering any base offset.
%       create_new_robot (Protected) - Create new robot graphics.
%       pose_jacobian - Compute the mobile base Jacobian while considering the base offset.
%       raw_fkm - Compute the mobile base pose without considering any base offset.
%       raw_pose_jacobian - Compute the mobile base Jacobian without considering the base offset.
%       set_base_diameter - Change the base diameter.
%       update_robot (Protected) - Move an existing graphical robot to a specified configuration.
%
%       See also DQ_MobileBase, DQ_DifferentialDriveRobot

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

classdef DQ_HolonomicBase < DQ_MobileBase
    
    properties (Access = protected)
        base_diameter = 0.5; %The default base diameter is 0.5 m
    end
    
    methods
        function obj = DQ_HolonomicBase()
            obj.dim_configuration_space = 3;            
        end
       
        function pose = raw_fkm(~, q)
        % RAW_FKM(q) returns the pose of a mobile base given the 
        % configuration q = [x,y,phi]'. It does not take into consideration
        % the base frame displacement (e.g., base height).
        %
        % TODO: Since this function uses as reference the global frame, and
        % implement the fkm with respect a reference frame.
            x = q(1);
            y = q(2);
            phi = q(3);
            
            include_namespace_dq
            
            real_part = cos(phi/2) + k_*sin(phi/2);
            dual_part = (1/2)*i_*(x*cos(phi/2) + y*sin(phi/2)) + ...
                        (1/2)*j_*(-x*sin(phi/2) + y*cos(phi/2));            
            pose = real_part + E_*dual_part;
        end
        
        function pose = fkm(obj, q)
        % FKM(q) returns the pose of a mobile base given the 
        % configuration q = [x,y,phi]'. It takes into consideration
        % the base frame displacement (e.g., base height).
            pose = obj.raw_fkm(q)*obj.frame_displacement;
        end
            
        function J = raw_pose_jacobian(~,q)
        % RAW_POSE_JACOBIAN(q) returns, given the configuration 
        % q = [x,y,phi]', the mobile-base pose Jacobian J that satisfies 
        % x_dot = J*q, where x_dot is the time derivative of the unit dual 
        % quaternion that represents the mobile-base pose. It does not take into
        % consideration the base frame displacement (e.g., base height).
            x = q(1);
            y = q(2);
            phi = q(3);
            
            s = sin(phi/2);
            c = cos(phi/2);
            
            j71 = -0.5*s;
            j62 = -j71;
            j13 = -j62;
            
            j72 = 0.5*c;
            j61 = j72;
            j43 = j61;
            
            j63 = 0.25*(-x*s + y*c);
            j73 = 0.25*(x*c - y*s);
            
            J = [0, 0, j13;
                 0, 0, 0;
                 0, 0, 0;
                 0, 0, j43;
                 0, 0, 0;
                 j61, j62, j63;
                 j71, j72, j73;
                 0, 0, 0];
        end
        
        function J = pose_jacobian(obj,q)
        % POSE_JACOBIAN(q) returns, given the configuration 
        % q = [x,y,phi]', the mobile-base pose Jacobian J that satisfies 
        % x_dot = J*q, where x_dot is the time derivative of the unit dual 
        % quaternion that represents the mobile-base pose. It takes into
        % consideration the base frame displacement (e.g., base height).
            J = haminus8(obj.frame_displacement)*obj.raw_pose_jacobian(q);
        end
        
        function set_base_diameter(obj,diameter)
            % Set the base diameter, in meters. This function must be
            % called before the first plot to take effect in the
            % visualization.
            obj.base_diameter = diameter;
        end
    end
    
    methods (Access = protected)
        function h = create_new_robot(obj, opt)
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
         
            h.scale = opt.scale;
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
            
            diameter = opt.scale*obj.base_diameter;
            
            % Draw a small circle representing the robot base
            h.robot = rectangle('Position',[0,0,diameter,diameter],...
                'Curvature', [1,1], 'FaceColor', 'y');
           
            % create base frame
            if opt.frame,
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
                
            end
        end
        
        function update_robot(robot, q)
            % Get the handle to the graphical robot. Since each kinematic robot
            % stores just one graphical handle, if we want to plot the same robot
            % in different views, we must declare different robots with the same
            % name. Otherwise, if just one robot is declared, but plotted in
            % different windows/views, the kinematic robot will store the handle of
            % the last view only.
            h = robot.handle;
            mag = h.scale*robot.base_diameter/2;
            diameter = h.scale*robot.base_diameter;
            % base = vec3(translation(robot.base));
           
            % Get the origin of the base frame
            x = q(1) -  diameter/2;
            y = q(2) -  diameter/2;

            % Update the coordinates of each frame along the kinematic chain. This 
            % updates the line drawing that represents the robot kinematic chain.
            set(h.robot,'Position', [x,y,diameter,diameter]);

            % display the wrist axes and labels   
            % compute the wrist axes, based on final link transformation plus the
            % tool transformation.
            if isfield(h, 'x')
                % get the end-effector pose (considering the final transformation given
                % by set_end_effector()
                t = robot.fkm(q);
                t1 = vec3(translation(t));%+[0;0;h.scale/100];        

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
    end
end

