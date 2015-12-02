%norm(dq) returns the dual scalar corresponding to the norm of dq

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

function ret = norm(dq)
    aux = dq'*dq;
    %Taking the square root of aux (to be compatible with the definition of quaternion norm)
    %This is performed based on the Taylor expansion.
    if(aux.P == 0)
        ret = DQ(0);
    else
        aux.q(1)=sqrt(aux.q(1));
        aux.q(5)=aux.q(5)/(2*aux.q(1));
        
        %Applying the threshold to the norm
        for i=1:8
            if (abs(aux.q(i)) < DQ.threshold)
                aux.q(i) = 0;
            end
        end
        ret = aux;
    end    
end