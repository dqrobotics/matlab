% Abstract superclass used to define concrete classes based on constrained setpoint control.
%
% DQ_KinematicConstrainedSetpointController Properties:
%   equality_constraint_matrix - matrix used in the equality constraint
%   equality_constraint_vector - vector used in the equality constraint
%   inequality_constraint_matrix - matrix used in the inequality constraint
%   inequality_constraint_vector - vector used in the inequality constraint
%   
% DQ_KinematicConstrainedSetpointController Methods:
%   add_equality_constraint - (ABSTRACT) Add constraint of type A*qdot = a
%   add_inequality_constraint - (ABSTRACT) Add inequality constraint of type B*qdot <= b    
% See also DQ_KinematicController.

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
    
    methods (Abstract)
        
        % Given the matrix B and the vector b of compatible dimensions, add
        % the linear (on the control inputs) equality constraint B*qdot = b
        % The constraint is not persistent, that is, it must be *always*
        % defined *before* the control signal is computed in order to be
        % taken into consideration.
        add_equality_constraint(obj,B, b);    
        
        % Given the matrix B and the vector b of compatible dimensions, add
        % the linear (on the control inputs) inequality constraint B*qdot <= b
        % The constraint is not persistent, that is, it must be *always*
        % defined *before* the control signal is computed in order to be
        % taken into consideration.
        add_inequality_constraint(obj,B, b);    
    end
end