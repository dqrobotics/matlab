% comau = DQ_COMAU returns a DQ_kinematics object using the modified
% Denavit-Hartenberg parameters of the COMAU SmartSiX robot

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

function comau = DQ_COMAU
   %% Definitions for DQ_kinematics
    comau_DH_theta=  [0, -pi/2, pi/2, 0, 0, 0, pi];
    comau_DH_d =     [-0.45, 0, 0, -0.64707, 0, -0.095, 0];
    comau_DH_a =     [0, 0.150, 0.590, 0.13, 0, 0, 0];
    comau_DH_alpha = [pi, pi/2, pi, -pi/2, -pi/2, pi/2, pi];
    comau_dummy =    [0,0,0,0,0,0,1];

    comau_DH_matrix = [comau_DH_theta;
        comau_DH_d;
        comau_DH_a;
        comau_DH_alpha;
        comau_dummy];

    comau = DQ_kinematics(comau_DH_matrix, 'modified');

end