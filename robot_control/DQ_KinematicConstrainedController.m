% Abstract superclass used to define concrete classes based on constrained setpoint control.
%
% DQ_KinematicConstrainedController Properties:
%   equality_constraint_matrix - matrix used in the equality constraint
%   equality_constraint_vector - vector used in the equality constraint
%   inequality_constraint_matrix - matrix used in the inequality constraint
%   inequality_constraint_vector - vector used in the inequality constraint
%   
% DQ_KinematicConstrainedController Methods:
%   set_equality_constraint - Add constraint of type A*qdot = a
%   set_inequality_constraint - Add inequality constraint of type B*qdot <= b    
% See also DQ_KinematicController,
%          DQ_TaskspaceQuadraticProgrammingController.

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

classdef DQ_KinematicConstrainedController< DQ_KinematicController
    
    properties (Access = protected)
        equality_constraint_matrix;
        equality_constraint_vector;
        inequality_constraint_matrix;
        inequality_constraint_vector;
    end
    methods
        function controller = DQ_KinematicConstrainedController(robot)
            controller = controller@DQ_KinematicController(robot);
            
            controller.equality_constraint_matrix = [];
            controller.equality_constraint_vector = [];
            controller.inequality_constraint_matrix = [];
            controller.inequality_constraint_vector = [];
        end
    end
    
    methods 
        function set_equality_constraint(obj,Aeq,beq)
            % Add equality constraint
            %
            % ADD_EQUALITY_CONSTRAINT(Aeq,beq) adds the constraint Aeq*u = beq,
            % where Aeq is the equality matrix, beq is the equality vector
            % and u is the control input.
            % The constraint is not persistent, that is, it must be *always*
            % defined *before* the control signal is computed in order to be
            % taken into consideration.
            obj.equality_constraint_matrix = Aeq;
            obj.equality_constraint_vector = beq;
        end
        
        function set_inequality_constraint(obj,A,b)
            % Add inequality constraint
            %
            % ADD_INEQUALITY_CONSTRAINT(A,b) adds the constraint A*u <= b,
            % where A is the inequality matrix, b is the inequality vector
            % and u is the control input.
            % The constraint is not persistent, that is, it must be *always*
            % defined *before* the control signal is computed in order to be
            % taken into consideration.
            obj.inequality_constraint_matrix = A;
            obj.inequality_constraint_vector = b;
        end  
    end
end