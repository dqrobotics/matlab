classdef DQ
    % CLASS DQ
    % Define the operations that can be done with dual quaternions
    % Author: Bruno Vilhena Adorno. E-mail: adorno@ieee.org
    %
    %
    % Ways of defining a dual quaternion:
    %       dq = DQ %Create a dual quaternion 0+0*i+0*j+0*k+E*(0+0*i+0*j+0*k)
    %       dq = DQ(v) % Where is a 8-, 4-, or 1-dimension vector with
    %               the dual quaternions coefficients. The 4- and 1-dimension vectors are special cases,
    %               where the 4-dimension vector means quaternion definition,
    %               whilst the 1-dimension vector means a scalar, both in dual quaternion space.
    % 
    % Type help DQ.(method/constant/operation) for specific help.
    %
    % METHODS: 
    %       dq.P (or P(dq), the same applies to the other methods.)
    %       dq.D
    %       dq.Re
    %       dq.Im
    %       hamiplus4(h)
    %       haminus4(h)
    %       hamiplus8(h)
    %       haminus8(h)
    %       vec4(h)
    %       vec8(h)
    % CONSTANTS:
    %       DQ.E (dual unit)
    %       DQ.i (imaginary i)
    %       DQ.j (imaginary j)
    %       DQ.k (imaginary k)
    % BINARY OPERATIONS: +  (plus)
    %                    -  (minus)
    %                    *  (mtimes)
    %                    == (eq)
    %                    ~  (ne)
    %                    ^  (mpower), for the moment it is defined only for
    %                                 unit dual quaternions
    % UNARY OPERATIONS: inv
    %                    ' (conjugate)
    %
    %
    % FUNCTIONS APPLIED ONLY TO UNIT DUAL QUATERNIONS:
    %                    exp
    %                    is_unit
    %                    log
    %                    plot 
    %                    translation
    %                    rotation_axis
    %                    rotation_angle
    %                    translation
    %                    T
   
    properties
        %dual quaternion vector
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
    end
    
   
    methods
        function obj = DQ(v)
            if nargin == 0
                obj.q = [0;0;0;0;0;0;0;0];
            elseif isa(v,'DQ')
                obj = v;
            else
                if(size(v,2)~=1)
                    v=v';
                end
                
                if(length(v) == 8)
                    obj.q = v;
                elseif(length(v) == 4) %It's a quaternion
                    obj.q = [v;zeros(4,1)];
                elseif(size(v,1) == 1) %It's a scalar
                    obj.q = [v;zeros(7,1)];
                else
                    error('You must input vector with 1, 4, or 8 elements');
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
            warning('Function theta(x) deprecated. Please use instead rotation_angle(x)');
            res = rotation_angle(obj);
           
        end
        
       
        
        function G = gen(dquat)
            dq = DQ(dquat);
            psi_l = [dq.q(5)  dq.q(6)  dq.q(7) dq.q(8);
                     dq.q(6) -dq.q(5)  dq.q(8) -dq.q(7);
                     dq.q(7) -dq.q(8) -dq.q(5) dq.q(6);
                     dq.q(8)  dq.q(7) -dq.q(6) -dq.q(5);
                    ];
            omega_l = haminus4(P(dq)');
            psi_r = omega_l;
            
            G = 2*[psi_l psi_r;
                   omega_l zeros(4,4)];            
           
        end
       
        
        function res = tplus(obj) 
           warning('Function tplus(x) deprecated. Please use instead T(x)');
          
            res = T(obj);
        end
    end
end