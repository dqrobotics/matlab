% CLASS DQ - Units, operations and operators related to dual quaternions.
%
% Ways of defining a dual quaternion:
%
% FIRST (closer to mathematical notation):     
%       >> dq = 1 + 2 * DQ.i + 5 * DQ.E * DQ.k creates the dual quaternion
%          (1+2i)+E*(5k) in terms of its imaginary components and the dual unit. 
%       
% ALTERNATIVELY, 'include_namespace_dq' can be used, therefore one can write
%       >> dq = 1 + 2 * i_ + 5 * E_ * k_
% 
% SECOND (Matlab style):
%       >> dq = DQ(v), where v is a vector of 8, 6, 4, 3 or 1 dimensions with
%               the dual quaternions coefficients. 
%       More specifically:
%               1) An eight-dimensional v contains the coefficients of general
%               dual quaternions.
%               2) A six-dimensional v contains the coefficientes of a pure dual
%               quaternion
%               3) A four-dimensional v contains the coefficients of a general
%               quaternion
%               4) A three-dimensional v contains the coefficients of a pure
%               quaternion
%               5) An one-dimensional v contains the coefficient of a real
%               number.
% 
%   DQ Properties:
%       E - dual unit
%       i - imaginary i
%       j - imaginary j
%       k - imaginary k
%       C8 - conjugator matrix associated to vec8
%       C4 - conjugator matrix associated to vec4
%
%   DQ Methods (binary operations and relations):
%       + (plus) - dual quaternion addition
%       -(minus) - dual quaternion subtraction
%       * (mtimes) - dual quaternion multiplication
%       .* (times) - dual quaternion decompositional multiplication
%       / (mrdivide) - dual quaternion right division
%       \ (mldivide) - dual quaternion left division
%       ./ (rdivide) - dual quaternion right division under decompositional multiplication 
%       .\ (ldivide) - dual quaternion left division under decompositional multiplication
%       == (eq) - return true if two dual quaternions are equal, false otherwise
%       ~= (ne) - return true if two dual quaternions are different, false otherwise
%       ^  (mpower) - exponentiation of quaternions
%       Ad - adjoint operation
%       Adsharp - sharp adjoint operation
%       cross - cross product between two pure dual quaternions
%       dot - dot product between two pure dual quaternions
%
%   DQ Methods (unary operations):
%       ' (ctranspose) - dual quaternion conjugate 
%       .' (transpose) - dual quaternion sharp conjugate
%       conj - _same_ as '
%       inv - dual quaternion inverse
%       dinv - dual quaternion inverse under decompositional multiplication
%       P, D, Re, Im - returns the primary, dual, real, and imaginary parts, respectively
%       sharp - _same_ as .'
%
%   DQ Methods (general):
%       double - functional casting of real dual quaternions to double
%       is_line - verify if a dual quaternion is a line 
%       is_plane - verify if a dual quaternion is a plane
%       is_pure - verify if a dual quaternion is pure
%       is_pure_quaternion - verify if a quaternion is pure
%       is_quaternion - verify if it the dual part is zero
%       is_real - verify if the imaginary part is zero
%       is_real_number - verify if both dual and imaginary components are zero
%       is_unit - verify if a unit dual quaternion as unit norm
%       norm - return the norm of dual quaternion
%       logical - return true if different from zero, false otherwise
%       normalize - normalize a dual quaternion
%       plot - draw primitives represented by dual quaternions
%
%       DQ Methods (vector operations on dual quaternions)
%       crossmatrix4 -  map a pure quaternion into an expanded skew-symmetric matrix
%       hamiplus4, haminus4, hamiplus8, haminus8 - return the Hamilton operators
%       Q8 - return the partial derivative of the unit dual quaternion x with respect to log(x)
%       vec3, vec4, vec6, vec8 - map quaternions and dual quaternions to vectors
%     
%       DQ Methods (unit dual quaternions)
%       exp - dual quaternion exponential
%       log - dual quaternion logarithm
%       rotation - return a rotation component of a dual quaternion (unit-norm quaternion)
%       rotation_axis - return the rotation axis (pure quaternion)
%       rotation_angle - return the rotation angle (real scalar)
%       translation - return the translation quaternion (pure quaternion)
%       T - return the translational part represented by dual quaternions
%
%       See also DQ_Kinematics, DQ_KinematicController

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
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br


classdef DQ
    properties
        % dual quaternion vector (in future versions this variable will 
        % be renamed)
        q;
        % Internal properties for handling the graphical representation of
        % the dual quaternion
        
        %For internal use only
        handle_axis;
        
        %For internal use only
        handle_text;
        
        %For internal use only
        isplot;
    end
    
    properties (Constant)
        %Dual unit
        E = DQ([0,0,0,0,1,0,0,0]);
        %Imaginary i
        i = DQ([0,1,0,0]);
        %Imaginary j
        j = DQ([0,0,1,0]);
        %Imaginary k
        k = DQ([0,0,0,1]);
        %Absolute values below the threshold are considered zero. This
        %threshold is used in the following functions: display, norm, ne,
        %and eq.
        threshold = 1e-12;
        % Given the dual quaternion x, the matrix C8 is the one that
        % satisfies the relation vec8(x') = C8 * vec8(x)
        C8 = diag([1, -1, -1, -1, 1, -1, -1, -1]);
        % Given the quaternion x, the matrix C4 is the one that
        % satisfies the relation vec4(x') = C4 * vec4(x)
        C4 = diag([1, -1, -1, -1]);
    end
    
   
    methods
        function obj = DQ(v)
            if nargin == 0
                obj.q = [0;0;0;0;0;0;0;0];
            elseif isa(v,'DQ')
                obj = v;
            else
                % All vectors should be column vectors
                if(size(v,2)~=1)
                    v=v';
                end
                
                % The constructor acts as the mapping between $R^n$ and the
                % set of dual quaternions:                
                if(length(v) == 8)
                    obj.q = v;
                elseif(length(v) == 6) % It's a pure dual quaternion
                    obj.q = [0;v(1:3);0;v(4:6)];
                elseif(length(v) == 4) % It's a quaternion
                    obj.q = [v;zeros(4,1)];
                elseif(length(v) == 3) % It's a pure quaternion
                    obj.q = [0;v;zeros(4,1)];
                elseif(size(v,1) == 1) % It's a scalar
                    obj.q = [v;zeros(7,1)];
                else
                    error(['The DQ constructor accepts only vectors with 1' ... 
                        ' (scalar), 3 (pure quaternion), 4 (quaternion), 6' ...
                        ' (pure dual quaternion) or 8 (dual quaternion)'...
                        ' elements.']);
                end
            end
        end
        
        function val = P(s,elem)
            % dq.P returns the primary part of the dual quaternion.
            % dq.P(index), returns the coefficient corresponding to index
            if nargin == 2
                if elem > 4 || elem < 1
                    error('Index out of bonds');
                end
                val = s.q(elem);
            else
                val = DQ(s.q(1:4));
            end
            
        end
        
        
        function val = D(s,elem)
            % dq.D returns the dual part of the dual quaternion.
            % dq.D(index), returns the coefficient corresponding to index
             if nargin == 2
                if elem > 4 || elem < 1
                    error('Index out of bonds');
                end
                val = s.q(elem+4);
            else
               val = DQ([s.q(5:8)']);
            end
            
        end
        
        function val = Re(s)
            % Return the real part of the dual quaternion
            val = DQ([s.q(1),0,0,0,s.q(5),0,0,0]);
        end
        
        function val = Im(s)
            % Return the imaginary part of the dual quaternion
            val = DQ([0,s.q(2),s.q(3),s.q(4),0,s.q(6),s.q(7),s.q(8)]);
        end
        
        function H = hamiplus4(q)
            % Return the positive Hamilton operator (4x4) of the quaternion
            % q
            q = DQ(q);
            v = q.q;
            H = [ v(1) -v(2) -v(3) -v(4);
                  v(2)  v(1) -v(4)  v(3);
                  v(3)  v(4)  v(1) -v(2);
                  v(4) -v(3)  v(2)  v(1)];
        end
        
        function H = haminus4(q)
            % Return the negative Hamilton operator (4x4) of the quaternion
            % q
            q = DQ(q);
            v = q.q;
            H = [ v(1) -v(2) -v(3) -v(4);
                  v(2)  v(1)  v(4) -v(3);
                  v(3) -v(4)  v(1)  v(2);
                  v(4)  v(3) -v(2)  v(1)];
        end
        
        function H = hamiplus8(dq)
            % Return the positive Hamilton operator (8x8) of the dual quaternion
            % dq
            h = DQ(dq);
            
            H = [hamiplus4(h.P), zeros(4,4);
                 hamiplus4(h.D), hamiplus4(h.P)];
        end
        
        function H = haminus8(dq)
            % Return the negative Hamilton operator (8x8) of the dual quaternion
            % dq
            h = DQ(dq);
            H = [haminus4(h.P), zeros(4,4);
                 haminus4(h.D), haminus4(h.P)];
        end
        
        function v = vec3(dq)
            % Return the vector with the coefficients of the imaginary part 
            % (three coefficients)
            v = dq.q(2:4);
        end
        
        function v = vec6(dq)
            % Return the vector with the coefficients of the imaginary part 
            % (six coefficients)
            v = [dq.q(2:4);dq.q(6:8)];
        end
        
        function v = vec4(dq)
            % Return the vector with the quaternion coefficients (four
            % coefficients)
            dq = DQ(dq);
            v = dq.q(1:4);
        end
        
        function v = vec8(dq)
            % Return the vector with the dual quaternion coefficients
            % (eight coefficients).
            dq = DQ(dq);
            v = dq.q;
        end
        
      
        %% Deprecated functions. They are here in order to ensure compatibility with old scripts.
        %  However, they are likely to disappear in the next versions. Use
        %  them at your own risk!        
        function val = n(s,elem)
            %DEPRECATED. Please use P instead.
            warning('Function n deprecated. Please use P(x) instead');
            if nargin == 2
                if elem > 4 || elem < 1
                    error('Index out of bonds');
                end
                val = s.q(elem);
            else
                val = s.q(1:4);
            end
        end
        
        function val = d(s,elem)
            % Return the dual part of the dual quaternion
            warning('Function d deprecated. Please use D(x) instead');
            if nargin == 2
                if elem > 4 || elem < 1
                    error('Index out of bonds');
                end
                val = s.q(elem+4);
            else
                val = s.q(5:8);
            end
        end
        
        
        function val = t(s,elem)
            %DEPRECATED
            % Return the translation corresponding to the unit dual
            % quaternion, assuming a translation followed by rotation
            % movement.
            warning('Function t deprecated. Please use instead translation(x)');
            aux = 2*DQ(s.d)*DQ(s.n)';
            if nargin == 2
                if elem > 4 || elem < 1
                    error('Index out of bonds');
                end
                val = aux.n(elem);
            else
                val = aux.n;
            end
        end
        
        function val = vec(s,num)
            %DEPRECATED
            % dq.vec(index) %Return the cartesian vector corresponding to the
            % non-dual (index=1) or dual part (index=2)
            warning('Function vec deprecated');
            if num == 1
                val = s.q(2:4);
            elseif num == 2
                val = s.q(6:8);
            else
                error('Accepted values: 1 for the non-dual part and 2 for the dual part');
            end
        end
        
        
        function res = theta(obj)
            %DEPRECATED
            % Function theta(x) deprecated. Please use instead rotation_angle(x)
            warning('Function theta(x) deprecated. Please use instead rotation_angle(x)');
            res = rotation_angle(obj);
           
        end
        
       
        function res = G(h)
            %Given a unit dual quaternion x, a pure dual quaternion xi = w + DQ.E*v, 
            %where w is the angular velocity and v is the linear velocity, and considering x_dot as the time
            %derivative of x, then G
            %is the matrix that satisfies vec(xi)=G*vec(x_dot).
            %For more informations, see 
            %        B. V. Adorno, "Two-arm Manipulation: From Manipulators to Enhanced Human-Robot Collaboration 
            %          [Contribution ? la manipulation ? deux bras : des manipulateurs ? la collaboration homme-robot]," 
            %          Universit? Montpellier 2, 2011.
            dq = DQ(h);
            omega_l = 2*haminus4(P(dq)');
            omega_r = zeros(4);
            
            psi_l = 2*hamiplus4(D(dq))*DQ.C4;
            psi_r = omega_l;
            
            res = [omega_l omega_r;
                   psi_l psi_r];            
           
        end
       
        
        function res = tplus(obj) 
           %DEPRECATED
           % Function tplus(x) deprecated. Please use instead T(x)
           warning('Function tplus(x) deprecated. Please use instead T(x)');
          
            res = T(obj);
        end
    end
end