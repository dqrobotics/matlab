% (C) Copyright 2022 DQ Robotics Developers
%
% This file is part of DQ Robotics.
%
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published
%     by the Free Software Foundation, either version 3 of the License, or
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
%
%   1. Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp) 
%      Responsible for the original implementation.
%
classdef DQ_JointType <  uint32

   % This class is based on Table 1 of  Silva, Quiroz-Omaña, and Adorno (2022).
   % Dynamics of Mobile Manipulators Using Dual Quaternion Algebra
   % https://doi.org/10.1115/1.4054320
   enumeration
      REVOLUTE    (1)
      PRISMATIC   (2)
      SPHERICAL   (3)
      CYLINDRICAL (4)
      PLANAR      (5)
      SIX_DOF     (6)
      HELICAL     (7)
   end
end



