% Abstract class that defines a control law based on quadratic programming.
%
% Although there are inumerous controllers based on quadratic programming,
% this class is suitable for those whose objective function is based on
% task-space variables, such as the robot Jacobian and the task-space
% error.
%
% DQ_TaskspaceQuadraticProgrammingController Methods:
%   compute_objective_function_symmetric_matrix - (Abstract) Compute the matrix H used in the objective function qdot'*H*qdot + f'*qdot.
%   compute_objective_function_linear_component - (Abstract) Compute the vector f used in the objective function qdot'*H*qdot + f'*qdot.
%   compute_setpoint_control_signal - Based on the task setpoint, compute the control signal.
%   compute_tracking_control_signal - Based on the task trajectory, use the feedforward to compute the control signal.
% See also DQ_KinematicController, DQ_ClassicQPController.

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

classdef DQ_TaskspaceQuadraticProgrammingController < DQ_KinematicConstrainedController
    properties (Access = protected)
        solver;
    end
    
    methods (Abstract)
        compute_objective_function_symmetric_matrix(controller, J, task_error);
        compute_objective_function_linear_component(controller, J, task_error);
    end
    
    methods
        function controller = DQ_TaskspaceQuadraticProgrammingController(robot, solver)
            controller = controller@DQ_KinematicConstrainedController(robot);
            controller. solver = solver;
        end        
              
        function u = compute_setpoint_control_signal(controller, q, task_reference)
            % Based on the task reference, compute the control signal
            if controller.is_set()
                % get the task variable according to the control objective
                task_variable = controller.get_task_variable(q);
                % get the Jacobian according to the control objective
                J = controller.get_jacobian(q);

                % calculate the Euclidean error
                task_error = task_variable - task_reference;
                
                % calculate the parameters that quadprog use to solve the 
                % quadratic problem min 0.5 * norm(J*u+gain*task_error)^2 + 0.5*norm(u)^2 
                A = controller.inequality_constraint_matrix;
                b = controller.inequality_constraint_vector;
                Aeq = controller.equality_constraint_matrix;
                beq = controller.equality_constraint_vector;
                
                % compute the quadratic component of the objective function
                H = controller.compute_objective_function_symmetric_matrix(J, task_error);
                
                % compute the linear component of the objective function
                f = controller.compute_objective_function_linear_component(J, task_error);
                
                u = controller.solver.solve_quadratic_program(H,f,A,b,Aeq,beq);
                
                % verify if the closed-loop system has reached a stable
                % region and update the appropriate flags accordingly.
                controller.verify_stability(task_error);
                
                % Store the values of the last error signal and last
                % control signal
                controller.last_control_signal = u;
                controller.last_error_signal = task_error;
                
            end
        end
        
        function u = compute_tracking_control_signal(controller, q, ...
                task_reference, feedforward)
            if controller.is_set()
                % get the task variable according to the control objective
                task_variable = controller.get_task_variable(q);
                % get the Jacobian according to the control objective
                J = controller.get_jacobian(q);

                % calculate the Euclidean error
                task_error = task_variable - task_reference;
                
                % calculate the parameters that quadprog use to solve the 
                % quadratic problem min 0.5 * norm(J*u+gain*task_error)^2 + 0.5*norm(u)^2 
                A = controller.inequality_constraint_matrix;
                b = controller.inequality_constraint_vector;
                Aeq = controller.equality_constraint_matrix;
                beq = controller.equality_constraint_vector;
                
                % compute the quadratic component of the objective function
                H = controller.compute_objective_function_symmetric_matrix(J,...
                    task_error - (1/controller.gain)*feedforward);
                
                % compute the linear component of the objective function
                f = controller.compute_objective_function_linear_component(J,...
                    task_error - (1/controller.gain)*feedforward);
                
                u = controller.solver.solve_quadratic_program(H,f,A,b,Aeq,beq);
                
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
end