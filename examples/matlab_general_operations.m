% Show the main operations used in the class DQ and basic dual quaternion
% algebra. 

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

function matlab_general_operations(varargin)
% TODO: add C8, ~, exp, is_unit, log, T, cross, dot.
close all;
clc;
index = 1;
include_namespace_dq;

if isempty(varargin)
    fprintf(['\nType matlab_general_operations(stop), where stop = 0 if'...
        '\nyou want to print all commands immediately and stop = 1 if you'... 
        '\nwant to see the commands step-by-step. \n\n']);
    return;
else
    press_enter = cell2mat(varargin);
end

global dq;
global a;
global b;
global p;
global r;
global x;
dq = 1 + 2*i_ + 3*j_ + 4*k_ + E_*(5 + 6*i_ + 7*j_ + 8*k_);
a = i_ + E_*(1 + k_);
b = -2 + j_ + E_*(i_ + k_);
r = cos(pi/8) + i_*sin(pi/8);
p = i_ + j_ + k_;
x = r + E_*0.5*p*r;

command_list = {'dq1 = DQ' %1
                'dq1 = DQ; class(dq1)'
                'dq2 = 2*DQ(1), class(dq2)'
                'DQ.i, DQ.j, DQ.k'
                'DQ.E' %5
                'DQ.E*DQ.E'
                 ['1 + 2*DQ.i + 3*DQ.j + 4*DQ.k + DQ.E*(5 + 6*DQ.i +'...
    '7*DQ.j + 8*DQ.k)']
                ['dq = 1 + 2*i_ + 3*j_ + 4*k_ '...
                '+ E_*(5 + 6*i_ + 7*j_ + 8*k_)']
                'DQ([1,2,3,4,5,6,7,8])'
                'dq.P' %10
                'dq.D'
                'dq.Re'
                'dq.Im'
                'dq.P.Re'
                'dq.D.Im' %15
                'dq.P(2)'
                'dq.vec8'
                'dq.P.vec4, dq.D.vec4'
                'i_ * a' % 19.1
                'a  * i_'% 19.2
                'a + b'  % 20
                'a - b'
                'a * b'
                'hamiplus8(a)*vec8(b)'%23
                'haminus8(b)*vec8(a)'%23.1
                'a/b , a*inv(b), a\b, inv(a)*b'
                '-a'
                'a'''
                'a.'''
                'norm(b)'
                'inv(b)'
                'b*inv(b)'
                'norm(cos(pi/8) + i_*sin(pi/8))'
                'p = i_ + j_ + k_'      
                'x = r + E_*0.5*p*r'
                'norm(x)'
                'translation(x)'
                'translation(x) == p'
                'x.rotation_angle, pi/4'
                'x.rotation_axis'
                'plot(DQ(1)); hold on; plot(x);'
                };
           
comments = {'Let us define a zero dual quaternion' %1
            'Now we verify that dq1 is indeed a dual quaternion and not a double'
            'A double multiplied by a DQ results in a DQ.'
            'The imaginary units are given by DQ.i, DQ.j, DQ.k'
            'The dual unit is available too.' %5
            'Remember that the dual unit is nilpotent, that is, DQ.E*DQ.E = 0.'
            ['We can declare dual quaternions using the imaginary and dual'...
            ' units']
            ['Alternatively, we can use the include_dq_namespace to use '...
            'the shortcuts i_,j_,k_,E_']
            ['We can also use a vector containing the dual quaternion '...
            'coefficients']
            'Retrieving the primary part of dq' %10
            'Retrieving the dual part of dq'
            'Retrieving the real part of dq'
            'Retrieving the imaginary part of dq'
            'Retrieving the real part of the primary part of dq'
            'Retrieving the imaginary part of the dual part of dq' %15
            ['Retrieving the coefficient that multiplies the DQ.i unit in the '...
            'primary part']
            ['Using vec8 to put the dual quaternion coefficients into a '...
            'eight-dimensional vector']
            ['Using vec4 to put the quaternion coefficients into a '...
            'four-dimensional vector']
            ['Recall that i_ * i_ = -1 and i_ * k_ = -j = -k_ * i; ' ...
            'therefore, if a = i_ + E_*(1 + k_), then'] %19.1
            'and' %19.2
            ['We can use the + and * operations in the usual way. '...
            'Consider a = i_ + E_*(1 + k_) and b = -2 + j_ + E_*(i_ + k_).'...
            'Therefore, a + b'] %20
            'a - b'
            'a * b'
            ['The same result can be found using vector algebra and Hamilton'...
            ' operators. For instance vec8(a*b) is given by'] %23
            'which is equivalent to'%23.1
            ['Right and left divisions work as well: a/b is equivalent to '...
            'a*inv(b) and a\b is equivalent to inv(a)*b']%23.2
            'Given a = i_ + E_*(1 + k_), then -a'
            'Conjugate of a = Re(a) + Im(a): a''= Re(a) - Im(a)' %25
            'Sharp conjugate of a = a.P + E_ * a.D: a.'' = a.P - E_ * a.D'
            'Calculate the dual quaternion norm of b = -2 + j_ + E_*(i_ + k_)'
            'Calculate the inverse of b = -2 + j_ + E_*(i_ + k_)'
            'b*inv(b) = inv(b)*b = 1'
            ['Let us define a unit quaternion r = cos(pi/8) + i_*sin(pi/8)'...
            ' that represents a rotation of pi/4 rad around the x-axis']
            'Let us define the pure quaternion p = i_ + j_ + k_'
            'Let us define the unit dual quaternion x = r + E_*0.5*p*r'
            'x indeed has unit norm:'
            'Retrieve the translation from x'
            'We can use the == to compare if two dual quaternions are equal'
            'Retrieve the rotation angle'
            'Retrieve the rotation axis'
            ['Now it''s time to go visual. First we plot the reference '...
            'frame and then we plot x']
            };
            
           
max_index = length(command_list);
           
for i =  1:length(command_list) 
    fprintf('\n---------------------------------------------\n%s',comments{i});
    fprintf('\n\n%d of %d) ',index,max_index);
    
    cmd(command_list{i});
    if press_enter
        fprintf('\nPress <ENTER> to continue.\n');
        pause();
    end
    index = index + 1;
end

end

function cmd(text)
global dq;
global a;
global b;
global r;
global p;
global x;
include_namespace_dq;
fprintf('COMMAND: %s\n', text);
eval(text);
end




