% Abstract superclass used to define concrete classes based on setpoint control.
%
% DQ_KinematicSetpointController Methods:
%    compute_control_signal - (ABSTRACT) compute the control signal for the regulation problem.
%
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

classdef DQ_KinematicSetpointController < DQ_KinematicController
    methods
        function controller = DQ_KinematicSetpointController(robot)
            controller = controller@DQ_KinematicController(robot);
        end
    end
    
    methods (Abstract)
        % Given the configuration vector q and the task_reference vector,
        % compute the control signal assuming the regulation problem, that
        % is, the controller does not explicitly take into account any
        % time-varying reference.
        control_signal = compute_control_signal(obj,q, task_reference);        
    end
end