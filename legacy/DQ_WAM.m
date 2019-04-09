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


function wam = DQ_WAM
%Standard D-H of WAM arm
    wam_DH_theta=[0, 0, 0, 0, 0, 0, 0];
    wam_DH_d = [0, 0, 0.55, 0, 0.3, 0, 0.0609];
    wam_DH_a = [0, 0, 0.045, -0.045, 0, 0, 0];
    wam_DH_alpha =  pi*[-0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0];
    wam_DH_matrix = [wam_DH_theta;
                      wam_DH_d;
                      wam_DH_a;
                      wam_DH_alpha];

    wam = DQ_kinematics(wam_DH_matrix,'standard');
end