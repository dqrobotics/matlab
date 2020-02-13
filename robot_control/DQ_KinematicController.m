% Abstract superclass used to define task-space kinematic controllers.
%
% DQ_KinematicController Properties:
%   attached_primitive - Controlled primitive attached to the end-effector.
%   control_objective - Control objective given by ControlObjective enum.
%   gain - Default gain is zero to force the user to choose a gain.
%   last_control_signal - Last value computed for the control signal.
%   last_error_signal - Last value computed for the error signal.
%   stability_counter - Counter that is incremented for each iteration wherein the error variation is below the stability threshold.
%   stability_counter_max - The system is considered to have reached the stable region only after the stability_counter has its maximum value, namely stability_counter_max.
%   stability_threshold - Threshold to determine if a stable region has been reached.
%   robot - DQ_Kinematics object related to the robot to be controlled.
%   system_reached_stable_region_ - True if reached a stable region, false otherwise.
%   target_primitive - Target primitive to where the end-effector must converge.
%
% DQ_KinematicController Methods:
%   compute_setpoint_control_signal - (ABSTRACT) Compute the control input to regulate to a setpoint.
%   compute_tracking_control_signal - (ABSTRACT) Compute the control input to track a trajectory
%   get_control_objective - Return the control objective.
%   get_damping - Return the current damping used to prevent kinematic singularities.
%   get_last_error_signal - Return the last error signal.
%   get_jacobian - Return the correct Jacobian based on the control objective.
%   get_task_variable - Return the task variable based on the control objective.
%   is_set - Verify if the controller is set and ready to be used.
%   reset_stability_counter - Reset the stability counter to zero.
%   set_stability_counter_max - Set the maximum value for the stability counter (default value is 10).
%   set_control_objective - Set the control objective using predefined goals in ControlObjective.
%   set_damping - Set the damping to prevent instabilities near singular configurations.
%   set_gain - Set the controller gain.
%   set_primitive_to_effector - Attach primitive to the end-effector.
%   set_stability_threshold - Set the threshold that determines if a stable region has been reached.
%   set_target_primitive -  Set the primitive to where the robot must converge.
%   system_reached_stable_region - Return true if the system has reached a stable region, false otherwise.
%   verify_stability - Verify if the closed-loop region has reached a stable region.
%
% See also
%   DQ_KinematicConstrainedController,
%   DQ_PseudoinverseController,
%   DQ_TaskspaceQuadraticProgrammingController.


% (C) Copyright 2011-2019 DQ Robotics Developers
%
% This file is part of DQ Robotics.
%
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as
%     published by the Free Software Foundation, either version 3 of the
%     License, or (at your option) any later version.
%
%     DQ Robotics is distributed in the hope that it will be useful, but
%     WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%     Lesser General Public License for more details.
%
%     You should have received a copy of the GNU Lesser General Public
%     License along with DQ Robotics.  If not, see
%     <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br
classdef DQ_KinematicController < handle
    properties (Access=protected)
        % Controlled primitive attached to the end-effector. It is
        % initialized with zero and, in case the controller needs to use an
        % attached primitive that was not previously initialized, an error
        % is thrown.
        attached_primitive = DQ;
        
        % The controller must always be explicitly set.
        control_objective = ControlObjective.None; 
        
        % The default damping is zero.
        damping = 0.0;
        
        % Default gain is zero to force the user to choose a gain.
        gain = 0.0; 
        
        % True if reached a stable region, false otherwise.
        system_reached_stable_region_ = false; 
        
        % Last value computed for the control signal.
        last_control_signal; 
        
        % Last value computed for the error signal
        last_error_signal;
        
        % DQ_Kinematics object
        robot; 
        
        % If the stability threshold is never set to a value greater than
        % zero, the controller will run forever for all controllers that
        % ensure asymptotic stability.
        stability_threshold = 0;
        
        stability_counter = 0;
        
        stability_counter_max = 10;
        
        
        % When the goal is to make the end-effector converge to a target
        % primitive---namely a plane, a line, etc.---, the current target 
        % primitive coordinates must be explicitly defined in the variable
        % target_primitive
        target_primitive = DQ;
    end
    
    methods (Abstract) 
        % Compute the control input to regulate to a setpoint.
        %
        % COMPUTE_SETPOINT_CONTROL_SIGNAL(q,task_reference) calculates the
        % control signal to regulate the closed-loop system to a setpoint
        % given by task_reference, which depends on the control objective.
        compute_setpoint_control_signal(obj, q, task_reference);
        
        % Compute the control input to track a time-varying trajectory.
        %
        % COMPUTE_TRACKING_CONTROL_SIGNAL(q, task_reference, feedforward)
        % calculates the control signal to track a time-varying trajectory
        % given by task_reference, which depends on the control objective.
        % feedforward is the first time-derivative of task_reference.
        compute_tracking_control_signal(obj, q, task_reference, feedforward);
    end
    
    methods
        function obj = DQ_KinematicController(robot)
            if ~isa(robot,'DQ_Kinematics')
                error('DQ_KinematicController expects a DQ_Kinematics object');
            end
            obj.robot = robot;
            obj.last_control_signal = zeros(robot.get_dim_configuration_space(),1); 
        end
        
        function reset_stability_counter(obj)
        % Reset the stability counter to zero
            obj.stability_counter = 0;
        end
        
        function set_stability_counter_max(obj, max)
        % Set the maximum value for the stability counter (default value is 10)
            obj.stability_counter_max = max;
        end
            
            
        function set_primitive_to_effector(obj, primitive)
        % Attach primitive to the end-effector
        %
        % SET_PRIMITIVE_TO_EFFECTOR(primitive) attach the primitive to
        % be controlled to the end-effector. For example, if the goal is
        % to align a line collinear with the end-effector z-axis with a
        % line in the workspace, then primitive = k_. If the goal is to
        % align a plane coplanar to the yz-end-effector end-effector plane,
        % then primitive = i_.
            obj.attached_primitive = primitive;
        end
        
        
        
        function ret = get_control_objective(controller)
            % Return the control objective
            ret = controller.control_objective;
        end
        
        function ret = get_damping(controller)
            % Return the current damping used to prevent kinematic singularities.
            ret = controller.damping;
        end
        
        function J = get_jacobian(controller, q)
            % Return the correct Jacobian based on the control objective
            %
            % GET_JACOBIAN(q) returns the Jacobian
            % related to the ControlObjective value stored in
            % control_objective. The argument 'q' is the vector of joint
            % configurations.
            
            J_pose = controller.robot.pose_jacobian(q);
            x_pose = controller.robot.fkm(q);
            primitive = controller.attached_primitive;
            
            switch controller.control_objective
                case ControlObjective.Distance
                    J = controller.robot.distance_jacobian(J_pose, x_pose);
                    
                case ControlObjective.DistanceToPlane
                    if is_plane(controller.target_primitive)
                        plane = controller.target_primitive;
                    else
                        error(['Please set the target plane with the '...
                            'method set_target_primitive()']);
                    end
                    J_trans = controller.robot.translation_jacobian(J_pose, ...
                        x_pose);
                    p = translation(x_pose);
                    J = controller.robot.point_to_plane_distance_jacobian(...
                        J_trans, p, plane);
                    
                case ControlObjective.Line
                    if controller.attached_primitive
                        J = controller.robot.line_jacobian(J_pose, x_pose, ...
                                                                    primitive);
                    else
                        error('The primitive to be controlled is not set yet.');
                    end
                    
                case ControlObjective.Rotation
                    J = controller.robot.rotation_jacobian(J_pose);
                    
                case ControlObjective.Plane
                    if controller.attached_primitive
                        J = controller.robot.plane_jacobian(J_pose, x_pose, ...
                                                                    primitive);
                    else
                        error('The primitive to be controlled is not set yet.');
                    end
                    
                case ControlObjective.Pose
                    J = J_pose;
                    
                case ControlObjective.Translation
                    J = controller.robot.translation_jacobian(J_pose, x_pose);
                    
                case ControlObjective.None
                    error(['Set the control objective by using the'
                        'set_control_objective() method']);
            end
        end
        
        function ret = get_last_error_signal(controller)
            % Return the last error signal
            ret = controller.last_error_signal;
        end
        
        function task_variable = get_task_variable(controller, q)
            % Return the task variable based on the control objective
            %
            % GET_TASK_VARIABLE(q) returns the task variable related to the
            % ControlObjective value stored in control_objective. 'q' is the
            % vector of joint configurations and 'primitive' is an optional
            % argument that is used only in case the control_objective has one
            % of the following values: {Line, Plane};
            
            x_pose = controller.robot.fkm(q);
            primitive = controller.attached_primitive;
            
            switch controller.control_objective
                case ControlObjective.Distance
                    p = vec3(translation(x_pose));
                    task_variable = p'*p;
                    
                case ControlObjective.DistanceToPlane
                    if is_plane(controller.target_primitive)
                        plane = controller.target_primitive;
                    else
                        error(['Set the target plane with the method '...
                            'set_target_primitive()']);
                    end
                    
                    p = translation(x_pose);                    
                    task_variable = DQ_Geometry.point_to_plane_distance(p, ...
                        plane);
                    
                case ControlObjective.Line
                    task_variable = vec8(Ad(x_pose,primitive));
                    
                case ControlObjective.Rotation
                    task_variable = vec4(rotation(x_pose));
                    
                case ControlObjective.Plane
                    task_variable = vec8(Adsharp(x_pose, primitive));
                    
                case ControlObjective.Pose
                    task_variable = vec8(x_pose);
                    
                case ControlObjective.Translation
                    task_variable = vec4(translation(x_pose));
                    
                case ControlObjective.None
                    error(['Set the control objective by using the'
                        'set_control_objective() method']);
            end
        end
        
        function ret = is_set(controller)
            % Verify if the controller is set and ready to be used.

            if controller.control_objective == ControlObjective.None
                error(['Set the control objective by using the ' ...
                    'SET_CONTROL_OBJECTIVE() method']);
            elseif controller.gain == 0.0
                error('Set the control gain by using the SET_GAIN() method');
            else
                ret = true;
            end
        end
        
        function ret = system_reached_stable_region(controller)
        % Return true if the closed-loop system trajectories have converged to a 
        % positive invariant set (i.e., to a set from where they will not
        % leave anymore). In the case of set-point control, for instance,
        % it means that the task error will not decrease anymore.
            ret = controller.system_reached_stable_region_;
        end
        
        function ret = is_stable(controller)
        % Deprecated function that will be removed in version 20.04. It has
        % been replaced by system_reached_stable_region().
            warning(['DEPRECATED FUNCTION. It will be removed in version'...
                ' 20.04. Please use system_reached_stable_region() instead']);
            ret = controller.system_reached_stable_region_;
        end
                
        function set_control_objective(controller,control_objective)
        % Set the control objective using predefined goals in ControlObjective
            if isa(control_objective, 'ControlObjective')
                controller.system_reached_stable_region_ = false;
                
                controller.control_objective = control_objective;
                
                % Initialize the error signals according to the control
                % objective
                switch control_objective
                    case {ControlObjective.Distance, ...
                            ControlObjective.DistanceToPlane}
                        controller.last_error_signal = zeros(1);
                        
                    case {ControlObjective.Line, ...
                            ControlObjective.Plane, ...
                            ControlObjective.Pose}
                        controller.last_error_signal = zeros(8,1);
                        
                    case {ControlObjective.Rotation, ...
                            ControlObjective.Translation}
                        controller.last_error_signal = zeros(4,1);
                end
            else
                error(['Only objectives enumerated in ControlObjective are '
                    'allowed']);
            end
        end
        
        function set_damping(controller, damping)
            % Set the controller damping to prevent instabilities near
            % singularities.
            controller.damping = damping;
        end
        
        function set_gain(controller,gain)
            % Set the controller gain    
            controller.gain = gain;
        end
                
        function set_stability_threshold(controller,threshold)
        % Set the threshold that determines if a stable region has been reached.
            controller.stability_threshold = threshold;
        end
        
        function set_target_primitive(controller, primitive)
            % Set the primitive to where the robot must converge.
            %
            % set_target_primitive(primitive), where 'primitive' is the
            % primitive dual quaternion coordinates (i.e., dual quaternion
            % representing a plane, a line, etc.)
            controller.target_primitive = primitive;
        end
        
        function verify_stability(controller, task_error)
            % Verify if the closed-loop system has reached a stable region.
            %
            % If the task error changes below a threshold, then we consider
            % that the system has reached a stable region.
            
            if norm(controller.last_error_signal - task_error) < ...
                    controller.stability_threshold
                if controller.stability_counter < controller.stability_counter_max
                    controller.stability_counter = controller.stability_counter + 1;  
                end
            else
                % Even if the closed-loop system trajectories have already
                % reached a positive invariant set (i.e., a stable region),
                % it can be the case that the desired task set point
                % changes. In that case, the positive invariant set
                % changes, which implies that the the closed-loop system
                % trajectories have to reach this new positive invariant
                % set, that is, the new "stable region." Therefore, we need
                % to change the variable system_reached_stable_region to
                % false.
                controller.system_reached_stable_region_ = false;
                controller.stability_counter = 0;
            end
            
            if controller.stability_counter >= controller.stability_counter_max
                controller.system_reached_stable_region_ = true;
            end
        end
    end
end

