% Interface to Quadratic Program Solvers.
%
%   solve_quadratic_program - (Abstract) Solve a quadratic program.

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

classdef DQ_QuadprogSolver < DQ_QuadraticProgramSolver
    methods (Static)
        
        function u = solve_quadratic_program(H,f,A,b,Aeq,beq)
            % Turn-off quadprog messages
            options =  optimoptions('quadprog','Display', 'off');
            
            % compute the control signal u =
            u = quadprog(H,f,A,b,Aeq,beq,[],[],[],options);
        end
    end
end

