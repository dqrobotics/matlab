% Example of pseudoinverse-based controller with Euclidean error calculation.
%
% CLASSIC_PSEUDOINVERSE_CONTROLLER_EXAMPLE () Runs a simple example where a
% classic controller based on the pseudoinverse of the Jacobian matrix is
% used with an Euclidean error to control different tasks based on common
% geometric primitives (pose, plane, line, translation, rotation, and
% distance).

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

function classic_pseudoinverse_controller_example()

    % use the namespace
    include_namespace_dq
    
    % We use a KUKA LWR4 robot
    kuka = KukaLwr4Robot.kinematics();
    kuka.name = 'KUKA';

    % Initial configuration
    q = [0, 0.3770, 0.1257, -0.5655, 0, 0, 0]';
    % Integration step
    T = 0.001; 

    % The controller is based on the simple control law u =
    % pinv(J)*gain*task_error, where J is the robot Jacobian, gain
    % determines the convergence rate and task_error is the error between
    % the desired value and the current task variable. When the task error
    % derivative is below stability_threshold, the systems is said to have
    % reached a stable region.
    pseudoinverse_controller = DQ_PseudoinverseSetpointController(kuka);
    pseudoinverse_controller.set_gain(100);
    pseudoinverse_controller.set_stability_threshold(0.001);
    
    % Prepare the visualization
    figure;
    view(3);
    axis equal;
    hold on;
        
    % Let's define an alias for the method set_control_objective
    set_control_objective = @pseudoinverse_controller.set_control_objective;
    
    % The first task variable to be controlled is the end-effector pose
    set_control_objective(ControlObjective.Pose);
    
    % The purpose of this first WHILE is just to set the references and some
    % eye-candy visualization stuff. 
    while pseudoinverse_controller.get_control_objective() ~= ControlObjective.None
        switch pseudoinverse_controller.get_control_objective()
            case ControlObjective.Pose
                
                %%%%%%%%%%%% define reference for the controller %%%%%%%%%%%%%%%
                x_pose = kuka.fkm([1.7593, 0.8796, 0.1257, -1.4451,...
                                          -1.0053, 0.0628, 0]'); 
                task_reference = vec8(x_pose);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % choose the next objective to accomplish when this one is
                % fulfilled.
                control_objective = ControlObjective.Translation;
                
                %%%%%%%%%%%%%%%%%%%% begin eye candy %%%%%%%%%%%%%%%%%%%%%%%%%
                title('Controlling the pose.');
                % plot the reference to aid in the visualization
                handle_plot = plot(x_pose, 'scale', 0.2);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
            case ControlObjective.Translation
                
                %%%%%%%%%%%% define reference for the controller %%%%%%%%%%%%%%%
                task_reference = vec4(0*i_ + 0*j_ + 0*k_);
               
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % choose the next control objective
                control_objective = ControlObjective.Rotation;
                
                %%%%%%%%%%%%%%%%%%%%%% begin eye candy %%%%%%%%%%%%%%%%%%%%%%%%%
                title(['Controlling the translation. The end-effector must '...
                      'go to the origin.']);
                % delete the previous plot to have a cleaner figure
                if exist('handle_plot','var')
                    for i = 1:3
                        delete(handle_plot.handle_axis{i});
                        delete(handle_plot.handle_text{i});
                    end
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
            
            case ControlObjective.Rotation
                
                %%%%%%%%%%%% define reference for the controller %%%%%%%%%%%%%%%
                task_reference = vec4(DQ(1));        
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                

                % Choose next objective
                control_objective = ControlObjective.Distance;
                
                %%%%%%%%%%%%%%%%%%%%%% begin eye candy %%%%%%%%%%%%%%%%%%%%%%%%%
                title(['Controlling the rotation. The end-effector frame '...
                      'be aligned with the global reference frame.']);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
            case ControlObjective.Distance
                
                %%%%%%%%%%%% define reference for the controller %%%%%%%%%%%%%%%
                task_reference = 0.2^2; % The reference already is a 'vector'                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % Choose next objective
                control_objective = ControlObjective.Line;
                
                %%%%%%%%%%%%%%%%%%%%%% begin eye candy %%%%%%%%%%%%%%%%%%%%%%%%%
                title(['Controlling the distance between the end-effector '...
                      'and the origin of the global reference frame.']);
                  
                % Draw a sphere to better visualize what's happening. The
                % end-effector should converge to any point on the sphere
                % surface
                [X,Y,Z] = sphere;
                handle_plot = surf(0.2*X, 0.2*Y, 0.2*Z);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
            case ControlObjective.Line
                %%%%%%%%%%%% define reference for the controller %%%%%%%%%%%%%%%
                line_direction = normalize(DQ(rand(3,1)));
                line_point =  DQ(-0.3*ones(3,1)+ 0.6*rand(3,1));
                % The desired line is the first parameter
                line_d = line_direction + ...
                                        E_ * cross(line_point, line_direction);
                % The task reference is always mapped to a vector
                task_reference = vec8(line_d);
                % The line passing through the end-effector x-axis is the
                % second parameter. It'll be aligned with the desired line.                
                pseudoinverse_controller.attach_primitive_to_effector(i_);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % Choose next objective  
                control_objective = ControlObjective.Plane;
                
                %%%%%%%%%%%%%%%%%%%%%% begin eye candy %%%%%%%%%%%%%%%%%%%%%%%%%
                if exist('handle_plot','var')
                    delete(handle_plot); 
                end
                title(['Align a line that passes through the end-effector '...
                      'x-axis with a random black line in the workspace.']);
                  
                handle_plot = plot(line_d,'line',3, 'color', 'k');  
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
            case ControlObjective.Plane
                %%%%%%%%%%%% define reference for the controller %%%%%%%%%%%%%%%
                plane_normal = normalize(DQ(rand(3,1)));
                plane_point =  DQ(-0.3*ones(3,1)+ 0.6*rand(3,1));
                plane_d = -(plane_normal + E_ * dot(plane_point, plane_normal));
                % The reference is always mapped to a vector
                task_reference = vec8(plane_d); 
                % But the geometric primitive associated with the
                % end-effector is not. Furthermore, the goal is to align a
                % plane perpendicular to the robot end-effector's z-axis,
                pseudoinverse_controller.attach_primitive_to_effector(k_);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % Choose next objective 
                control_objective = ControlObjective.None;
                
                %%%%%%%%%%%%%%%%%%%%%% begin eye candy %%%%%%%%%%%%%%%%%%%%%%%%%
                if exist('handle_plot','var')
                    delete(handle_plot); 
                end
                
                plot(plane_d, 'plane', 3, 'color', 'yellow');
                
                title(['Controlling a plane that passes through the '...
                        'origin of the end-effector frame and with normal '...
                        'given by the end-effector z-axis.']);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        % This is actually the important part on how to use the controller. 
        while ~pseudoinverse_controller.is_stable()
            u = pseudoinverse_controller.compute_control_signal(q, ...
                                                                task_reference);
            q = q + T*u;
            plot(kuka,q,'nojoints');
            drawnow;
        end
        
        % Set the control objective for the next iteration
        set_control_objective(control_objective);
        % Pause three seconds to take a breath.
        pause(3);
    end
end




