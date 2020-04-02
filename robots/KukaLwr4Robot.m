% Create a new KUKA LW4 robot manipulator
% KukaLwr4Robot Methods (Static):
%   kinematics - Return a DQ_SerialManipulator object with the KUKA LW4 robot manipulator kinematic parameters

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

classdef KukaLwr4Robot
    methods (Static)
        function ret = kinematics()
            %Standard D-H of KUKA-LWR
            kuka_DH_theta=[0, 0, 0, 0, 0, 0, 0];
            kuka_DH_d = [0.310, 0, 0.4, 0, 0.39, 0, 0];
            kuka_DH_a = [0, 0, 0, 0, 0, 0, 0];
            kuka_DH_alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];
            kuka_DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);
            kuka_DH_matrix = [kuka_DH_theta;
                kuka_DH_d;
                kuka_DH_a;
                kuka_DH_alpha;
                kuka_DH_type];

            ret = DQ_SerialManipulatorDH(kuka_DH_matrix,'standard');
        end
    end
end
