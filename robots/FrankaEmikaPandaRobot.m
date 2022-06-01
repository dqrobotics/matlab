% (C) Copyright 2011-2022 DQ Robotics Developers
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
% DQ Robotics website: https://dqrobotics.github.io/
%
% Contributors to this file:
%     Juan Jose Quiroz Omana - juanjqogm@gmail.com

classdef FrankaEmikaPandaRobot
    methods (Static)
        function ret = kinematics()
            %Modified D-H of FrankaEmikaPanda
            franka_DH_theta=[0, 0, 0, 0, 0, 0, 0];
            franka_DH_d = [0.333, 0, 3.16e-1, 0, 3.84e-1, 0, 0]; %1.07e-1
            franka_DH_a = [0, 0, 0, 8.25e-2, -8.25e-2, 0, 8.8e-2];
            franka_DH_alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2];
            franka_DH_type =  double([repmat(DQ_JointType.REVOLUTE,1,7)]);
            franka_DH_matrix = [franka_DH_theta;
                franka_DH_d;
                franka_DH_a;
                franka_DH_alpha;
                franka_DH_type];

            ret = DQ_SerialManipulatorMDH(franka_DH_matrix);
            % There is a final transformation for the end-effector given by
            % a translation of d=1.07e-1 along around the local z-axis.
            ret.set_effector(1+DQ.E*0.5*DQ.k*1.07e-1);
            
            % If you want to describe the FrankaEmikaPanda robot using the
            % standard DH convention, there is no need to perform the
            % additional final transformation. In that case, the robot is
            % described as:
            % Standard D-H of FrankaEmikaPanda
            % franka_DH_theta=[0, 0, 0, 0, 0, 0, 0];
            % franka_DH_d = [0.333, 0, 3.16e-1, 0, 3.84e-1, 0, 1.07e-1];
            % franka_DH_a = [0, 0, 8.25e-2, -8.25e-2, 0, 8.8e-2, 0];
            % franka_DH_alpha = [-pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0];
            % franka_DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);
            % franka_DH_matrix = [franka_DH_theta;
            % franka_DH_d;
            % franka_DH_a;
            % franka_DH_alpha;
            % franka_DH_type];
            % ret = DQ_SerialManipulatorDH(franka_DH_matrix);
        end
    end
end