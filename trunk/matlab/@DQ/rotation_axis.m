%rotation_axis(dq) returns the rotation axis (nx*i + ny*j + nz*k) of the
%unit dual quaternion dq

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

function ret = rotation_axis(dq)
    dq = DQ(dq);
    
    if norm(dq) ~= 1
        error('The dual quaternion is non-unit');
    end
      
    phi = acos(dq.q(1));
        
    if(sin(phi) <= eps) %sin(pi) is not zero due to machine precision, hence if sin(phi) is smaller than eps (i.e., the spacing of floting point numbers) we consider it zero. 
        ret= DQ([0,0,0,1]); %This is just a convention. It could be any rotation axis.
    else        
        ret = dq.P.Im*(sin(phi)^(-1));
    end

end