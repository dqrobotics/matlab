% denso = DQ_DENSO returns a DQ_kinematics object using the modified
% Denavit-Hartenberg parameters of the denso SmartSiX robot

% (C) Copyright 2015 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br
%     Andressa Martins Oliveira

function denso = DQ_DENSO
   %% Definitions for DQ_kinematics
    denso_DH_theta=  [0,pi/2,-pi/2,0,0,0];
    denso_DH_d =     [0.125,0,0,0.210,0,0.070];
    denso_DH_a =     [0,0.210, -0.075, 0, 0,0];
    denso_DH_alpha = [pi/2,0,-pi/2, pi/2,-pi/2,0];
    

    denso_DH_matrix = [denso_DH_theta;
        denso_DH_d;
        denso_DH_a;
        denso_DH_alpha;
        ];

    denso = DQ_kinematics(denso_DH_matrix, 'standard');

end
