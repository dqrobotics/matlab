% IS_PLANE(x) receives a dual quaternion x and returns 1 if it is a plane
% (i.e., it has unit norm and x.Im.D = 0), and 0 otherwise.
% See also is_pure, is_pure_quaternion is_quaternion, is_real, is_real_number,
% is_unit

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
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br

function ret = is_plane(x)
    if is_unit(x) && is_real(D(x))
        ret = 1;
    else
        ret = 0;
    end
end
