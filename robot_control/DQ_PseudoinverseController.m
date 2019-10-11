% Implements a classic control law based on the Jacobian pseudoinverse and an Euclidean error.
%
% Usage: controller = DQ_PseudoinverseController(robot), where
% robot is a DQ_Kinematics object.
%
% DQ_PseudoinverseController Methods:
%   compute_setpoint_control_signal - Compute the control signal for a setpoint reference.
%   compute_tracking_control_signal - Compute the control signal for a time-varying trajectory.
%
% For more methods and properties, see also DQ_KinematicController.

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

classdef DQ_PseudoinverseController < DQ_KinematicController
    methods
        function controller = DQ_PseudoinverseController(robot)
            controller = controller@DQ_KinematicController(robot);
        end
        
        function u = compute_tracking_control_signal(controller, q, ...
                task_reference, feedforward)
            % Based on the task reference, compute the control signal
            if controller.is_set()
                % get the task variable according to the control objective
                task_variable = controller.get_task_variable(q);
                % get the Jacobian according to the control objective
                J = controller.get_jacobian(q);
                
                % calculate the Euclidean error
                task_error = task_variable - task_reference;
                % compute the control signal
                if(controller.damping == 0.0)
                    u = pinv(J)*(-controller.gain*task_error + ...
                        feedforward);
                else
                    u = inv(J'*J+controller.damping^2* ...
                        eye(length(q)))*J'*...
                        (-controller.gain*task_error + feedforward);
                end
                
                % verify if the closed-loop system has reached a stable
                % region and update the appropriate flags accordingly.
                controller.verify_stability(task_error);
                
                % Store the values of the last error signal and last
                % control signal
                controller.last_control_signal = u;
                controller.last_error_signal = task_error;
            end
        end
        
        function u = compute_setpoint_control_signal(controller, q, ...
                task_reference)
            u = controller.compute_tracking_control_signal(q, ...
                task_reference, zeros(size(task_reference)));
        end
    end
end