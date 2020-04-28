% (C) Copyright 2011-2019 DQ Robotics Developers
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

classdef Ax18ManipulatorRobot
    methods (Static)
        function ax = kinematics()
            %Standard D-H of AX18 arm
            ax18_DH_theta   = [0       0      -pi/2   -pi/2      -pi/2];  % theta
            ax18_DH_d       = [0.167   0       0     0.1225          0];  % d
            ax18_DH_a       = [0      0.159    0.02225    0      0.170];  % a
            ax18_DH_alpha   = [-pi/2   0      -pi/2   -pi/2          0];  % alpha
            ax18_DH_type    = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,5);
            
            ax18_DH_matrix = [  ax18_DH_theta;
                ax18_DH_d;
                ax18_DH_a;
                ax18_DH_alpha;
                ax18_DH_type;
                ]; % D&H parameters matrix for the arm model
            
            ax = DQ_SerialManipulatorDH(ax18_DH_matrix,'standard'); % Defines robot model using dual quaternions
        end
    end    
end