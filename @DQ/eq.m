% a == b returns 1 if the dual quaternion a equals the dual quaternion b, 
% otherwise returns 0
% See also ne

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

function ret = eq(a,b)
    
    if ~isa(a,'DQ');
        a = DQ(a);
    end
    if ~isa(b,'DQ');
        b = DQ(b);
    end

    ret = 1;
    % Verify if each coefficient of a is close enough to the corresponding
    % coefficient of b.
    for i=1:8
        if abs(a.q(i) - b.q(i)) > DQ.threshold
            ret = 0;
            break
        end
    end    
end