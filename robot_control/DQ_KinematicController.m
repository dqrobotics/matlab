% Abstract superclass used to define kinematic controllers.
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
%    get_control_objective - Return the control objective.
%    get_jacobian - Return the correct Jacobian based on the control objective.
%    get_task_variable - Return the task variable based on the control objective.
%    is_set - Verify if the controller is set and ready to be used.
%    set_control_objective - Set the control objective using predefined goals in ControlObjective.
%    set_gain - Set the controller gain.
%    set_stability_threshold - Set the threshold that determines if a stable region has been reached.
%    stable - Return true if the system has reached a stable region, false otherwise.
%

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
        robot; % DQ_Kinematics object
        control_objective = ControlObjective.None;
        gain = 0.0; % Default gain is zero to force the user to choose a gain.
        
        is_stable_ = false; % True if reached a stable region, false otherwise
        last_control_signal = 0; % Last value computed for the control signal.
        last_error_signal;
        stability_threshold = 0;
    end
    
    methods (Abstract)
        control_signal = compute_control_signal(obj);        
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
        
        function ret = get_control_objective(controller)
            % Return the control objective
            ret = controller.control_objective;
        end
        
        function J = get_jacobian(controller, q, primitive)
            % Return the correct Jacobian based on the control objective
            %
            % GET_JACOBIAN(q, primitive) returns the Jacobian related to the
            % ControlObjective value stored in control_objective. 'q' is the
            % vector of joint configurations and 'primitive' is an optional
            % argument that is used only in case the control_objective has one
            % of the following values: {Line, Plane};
            
            J_pose = controller.robot.pose_jacobian(q);
            x_pose = controller.robot.fkm(q);
            
            switch controller.control_objective
                case ControlObjective.Distance
                    J = controller.robot.distance_jacobian(J_pose, x_pose);
                    
                case ControlObjective.Line
                    J = controller.robot.line_jacobian(J_pose, x_pose, primitive);
                    
                case ControlObjective.Rotation
                    J = controller.robot.rotation_jacobian(J_pose);
                    
                case ControlObjective.Plane
                    J = controller.robot.plane_jacobian(J_pose, x_pose, primitive);
                    
                case ControlObjective.Pose
                    J = J_pose;
                    
                case ControlObjective.Translation
                    J = controller.robot.translation_jacobian(J_pose, x_pose);
                    
                case ControlObjective.None
                    error(['Set the control objective by using the'
                        'set_control_objective() method']);
            end
        end
        
        function task_variable = get_task_variable(controller, q, primitive)
            % Return the task variable based on the control objective
            %
            % GET_TASK_VARIABLE(q) returns the task variable related to the
            % ControlObjective value stored in control_objective. 'q' is the
            % vector of joint configurations and 'primitive' is an optional
            % argument that is used only in case the control_objective has one
            % of the following values: {Line, Plane};
            
            x_pose = controller.robot.fkm(q);
            
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

