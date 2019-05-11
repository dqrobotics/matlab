% Abstract superclass used to define task-space kinematic controllers.
%
% DQ_KinematicController Properties:
%   control_objective - Control objective given by ControlObjective enum.
%   gain - Default gain is zero to force the user to choose a gain.
%   is_stable - True if reached a stable region, false otherwise.
%   last_control_signal - Last value computed for the control signal.
%   last_error_signal - Last value computed for the error signal.
%   stability_threshold - Threshold to determine if a stable region has been reached.
%   robot - DQ_Kinematics object related to the robot to be controlled.
%
% DQ_KinematicController Methods:
%    attach_primitive_to_effector - Attach primitive to the end-effector.
%    get_control_objective - Return the control objective.
%    get_jacobian - Return the correct Jacobian based on the control objective.
%    get_task_variable - Return the task variable based on the control objective.
%    is_set - Verify if the controller is set and ready to be used.
%    is_stable - Return true if the system has reached a stable region, false otherwise.
%    set_control_objective - Set the control objective using predefined goals in ControlObjective.
%    set_gain - Set the controller gain.
%    set_stability_threshold - Set the threshold that determines if a stable region has been reached.
%    verify_stability - (ABSTRACT) Verify if the closed-loop region has reached a stable region.


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
        
        % Default gain is zero to force the user to choose a gain.
        gain = 0.0; 
        
        % True if reached a stable region, false otherwise.
        is_stable_ = false; 
        
        % Last value computed for the control signal.
        last_control_signal = 0; 
        
        % Last value computed for the error signal
        last_error_signal;
        
        % DQ_Kinematics object
        robot; 
        
        % If the stability threshold is never set to a value greater than
        % zero, the controller will run forever for all controllers that
        % ensure asymptotic stability.
        stability_threshold = 0;
    end
    
    methods (Abstract, Access = protected)
        verify_stability(obj, task_error);
    end
    
    methods
        function obj = DQ_KinematicController(robot)
            if ~isa(robot,'DQ_Kinematics')
                error('DQ_KinematicController expects a DQ_Kinematics object');
            end
            obj.robot = robot;
        end
        
        
        function attach_primitive_to_effector(obj, primitive)
        % Attach primitive to the end-effector
        %
        % ATTACH_PRIMITIVE_TO_EFFECTOR(primitive) attach the primitive to
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
        
        function J = get_jacobian(controller, q)
            % Return the correct Jacobian based on the control objective
            %
            % GET_JACOBIAN(q, primitive) returns the Jacobian related to the
            % ControlObjective value stored in control_objective. 'q' is the
            % vector of joint configurations and 'primitive' is an optional
            % argument that is used only in case the control_objective has one
            % of the following values: {Line, Plane};
            
            J_pose = controller.robot.pose_jacobian(q);
            x_pose = controller.robot.fkm(q);
            primitive = controller.attached_primitive;
            
            switch controller.control_objective
                case ControlObjective.Distance
                    J = controller.robot.distance_jacobian(J_pose, x_pose);
                    
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
                
        function set_control_objective(controller,control_objective)
        % Set the control objective using predefined goals in ControlObjective
            if isa(control_objective, 'ControlObjective')
                controller.is_stable_ = false;
                
                controller.control_objective = control_objective;
                
                % Initialize the error signals according to the control
                % objective
                switch control_objective
                    case ControlObjective.Distance
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
        
        function set_gain(controller,gain)
            % Set the controller gain    
            controller.gain = gain;
        end
                
        function set_stability_threshold(controller,threshold)
        % Set the threshold that determines if a stable region has been reached.
            controller.stability_threshold = threshold;
        end
        
        function ret = is_stable(controller)
        % Return true if the system has reached a stable region, false otherwise.    
            ret = controller.is_stable_;
        end
    end
end

