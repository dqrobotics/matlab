% Implements a control law based on quadratic programming.
%
% Usage: controller = DQ_QuadraticProgrammingController(robot), where
% robot is a DQ_Kinematics object.
%
% DQ_QuadraticProgrammingController Methods:
%   add_equality_constraint - Add the matrix B and the vector b to enforce the constraint B*qdot = b
%   add_inequality_constraint - Add the matrix B and the vector b to enforce the constraint B*qdot <= b
%   compute_control_signal - Based on the task reference, compute the control signal.
%   verify_stability - Verify if the closed-loop system has reached a stable region.
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

classdef DQ_QuadraticProgrammingController < DQ_KinematicConstrainedSetpointController
    methods
        function controller = DQ_QuadraticProgrammingController(robot)
            controller = controller@DQ_KinematicConstrainedSetpointController(robot);
        end        
        
        function add_equality_constraint(obj,B,b)
            obj.equality_constraint_matrix = B;
            obj.equality_constraint_vector = b;
        end
        
        function add_inequality_constraint(obj,B,b)
            obj.inequality_constraint_matrix = B;
            obj.inequality_constraint_vector = b;
        end
        
        function u = compute_control_signal(controller, q, task_reference)
            % Based on the task reference, compute the control signal
            if controller.is_set()
                % get the task variable according to the control objective
                task_variable = controller.get_task_variable(q);
                % get the Jacobian according to the control objective
                J = controller.get_jacobian(q);

                % calculate the Euclidean error
                task_error = task_variable - task_reference;
                
                % calculate the parameters that quadprog use to solve the 
                % quadratic problem min 0.5 * norm(J*u+gain*task_error)^2 
                A = controller.inequality_constraint_matrix;
                b = controller.inequality_constraint_vector;
                Aeq = controller.equality_constraint_matrix;
                beq = controller.equality_constraint_vector;
           
                H = J'*J + 0.00001*eye(controller.robot.get_dim_configuration_space());
                f = controller.gain*task_error'*J;
                
                % Turn-off quadprog messages
                options =  optimoptions('quadprog','Display', 'off');
                
                % compute the control signal
                u = quadprog(H,f,A,b,Aeq,beq,[],[],...
                    [],options);
                
                % verify if the closed-loop system has reached a stable
                % region and update the appropriate flags accordingly.
                controller.verify_stability(task_error);
                
                % Store the values of the last error signal and last
                % control signal
                controller.last_control_signal = u;
                controller.last_error_signal = task_error;
                
            end
        end
    end
    
    methods (Access = protected)
        function verify_stability(controller, task_error)
            % Verify if the closed-loop system has reached a stable region.
            %
            % If the task error changes below a threshold, then we consider
            % that the system has reached a stable region.
            % TODO: Choose different criteria
            
            if norm(controller.last_error_signal - task_error) < ...
                    controller.stability_threshold
                controller.is_stable_ = true;
            end
        end
    end
end