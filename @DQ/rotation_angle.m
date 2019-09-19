%ROTATION_ANGLE(X) returns the rotation angle of X or an error otherwise

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

function res = rotation_angle(x)
    if is_unit(x)
        cos_phi_over_2 = double(Re(P(x))); 
        % It can be the case that the dual quaternion has unit norm and equals
        % 1, according to the DQ Robotics precision, but it is slightly greater
        % than 1 (usually 1 + 1e-15). In that case, Matlab would return an 
        % imaginary number with a very small imaginary component. To
        % prevent that, we force the acos function to return a real number 
        res = real(2*acos(cos_phi_over_2));
    else
        error('The dual quaternion does not have unit norm.')
    end
end