% Decompositional multiplication between unit dual quaternions a and b, which 
% corresponds to the group operation of CMI(3).
%
% Given a = r_a + DQ.E*0.5*p_a*r_a and b = r_b + DQ.E*0.5*p_b*r_b,
% a .* b = T(a)*T(b)*P(a)*P(b), where T(a) = 1 + DQ.E*0.5*p_a and P(a) =
% r_a. Analogously, T(b) = 1 + DQ.E*0.5*p_b and P(b) = r_b.
%
% See ?Definition 1 of Adorno, B. V., & Fraisse, P. (2017). The cross-motion
% invariant group and its application to kinematics. IMA Journal of Mathematical 
% Control and Information, 34(4), 1359?1378. 
% https://doi.org/10.1093/imamci/dnw032

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

function res = times(a,b)
    if ~isa(a,'DQ');
        a = DQ(a);
    end
    if ~isa(b,'DQ');
        b = DQ(b);
    end
    
    % See DQ.T and DQ.P
    res = T(a)*T(b)*P(a)*P(b);
end