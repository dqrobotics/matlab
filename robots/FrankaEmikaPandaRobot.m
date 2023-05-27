% panda = FrankaEmikaPandaRobot.kinematics() returns a DQ_kinematics object
% using the modified Denavit-Hartenberg parameters of the Franka Emika
% Panda robot

% (C) Copyright 2011-2023 DQ Robotics Developers
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
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Frederico Fernandes Afonso Silva - frederico.silva@ieee.org

classdef FrankaEmikaPandaRobot
    methods (Static)
        function ret = kinematics()
            % Modified D-H of Franka Emika Panda
            mDH_theta = [0, 0, 0, 0, 0, 0, 0];
            mDH_d = [0.333, 0, 3.16e-1, 0, 3.84e-1, 0, 0];
            mDH_a = [0, 0, 0, 8.25e-2, -8.25e-2, 0, 8.8e-2];
            mDH_alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2];
            mDH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);
            
            mDH_matrix = [mDH_theta;
                          mDH_d;
                          mDH_a;
                          mDH_alpha;
                          mDH_type]; 
                        
            ret = DQ_SerialManipulatorMDH(mDH_matrix);

            % Set the base's reference frame
            xb = 1 + DQ.E*0.5*DQ([0, 0.0413, 0, 0]);
            ret.set_reference_frame(xb);
            ret.set_base_frame(xb);

            % Set the end-effector
            xe = 1 + DQ.E*0.5*DQ.k*1.07e-1;
            ret.set_effector(xe);
        end
    end
end