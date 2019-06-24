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

classdef KukaYoubot
    methods (Static)
        function robot = kinematics()
            % Mobile manipulator composed of a holonomnic mobile base 
            % serially coupled to a 5-DOF arm 
            
            % The DH parameters are based on Kuka's documentation:
            % http://www.youbot-store.com/wiki/index.php/YouBot_Detailed_Specifications
            % https://www.generationrobots.com/img/Kuka-YouBot-Technical-Specs.pdf
            
            pi2 = pi/2;
            arm_DH_theta = [    0,    pi2,       0,      pi2,        0];
            arm_DH_d =   [  0.147,      0,       0,        0,    0.218];
            arm_DH_a =   [      0,  0.155,   0.135,        0,        0];
            arm_DH_alpha =   [pi2,      0,       0,      pi2,        0];
            arm_DH_matrix = [arm_DH_theta;
                             arm_DH_d;
                             arm_DH_a;
                             arm_DH_alpha];
                    
            arm =  DQ_SerialManipulator(arm_DH_matrix,'standard');
            base = DQ_HolonomicBase();

            include_namespace_dq

            x_bm= 1 + E_*0.5*(0.22575*i_ + 0.1441*k_);

            base.set_frame_displacement(x_bm);

            robot = DQ_WholeBody(base);
            robot.add(arm);
        end
    end
end
