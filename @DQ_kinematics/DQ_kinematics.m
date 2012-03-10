% CLASS DQ_kinematics
% Usage: robot_kine = DQ_kinematics(A,convention), where:
% A is a 4 x n matrix containing the Denavit-Hartenberg parameters (n is the number of links)
%    A = [theta1 ... thetan;
%            d1  ...   dn;
%            a1  ...   an;
%         alpha1 ... alphan]
% convention is the convention used for the D-H parameters. Accepted values are
% 'standard' and 'modified'
%
% Type DQ_kinematics.(method,property) for specific help.
% Ex.: help DQ_kinematics.fkm
%
% METHODS:
%       fkm
%       jacobian
%       jacobp
%       jacobd
%
% PROPERTIES:
%       C8
%       C4


classdef DQ_kinematics
    properties
        links;
        theta,d,a,alpha;
        dummy;
        n_dummy;
        convention
    end
    
    properties (Constant)
        % Given the jacobian matrix J that satisfies
        % vec(dot_x) = J * dot_theta, where x is a dual quaternion, the
        % following relation is established:
        % vec(dot_x) = C8 * J * dot_theta
        C8 = diag([1, -1, -1, -1, 1, -1, -1, -1]);
        % Given the jacobian matrix J that satisfies
        % vec(dot_x) = J * dot_theta, where x is a quaternion, the following
        % relation is established: vec(dot_x) = C8 * J * dot_theta
        C4 = diag([1, -1, -1, -1]);
    end
    
    methods
        function obj = DQ_kinematics(A,type)
            if nargin == 0
                error('Input: matrix whose columns contain the DH parameters')
            elseif nargin==1
                obj.convention='standard';
            else
                obj.convention=type;
            end
            
            if size(A,1) == 5
                %There are dummy joints                
                obj.dummy = A(5,:);
                obj.n_dummy = sum(obj.dummy == 1);
                
            else                
                obj.dummy = zeros(1,size(A,2));
            end
            
            obj.links = size(A,2);
            obj.theta = A(1,:);
            obj.d = A(2,:);
            obj.a = A(3,:);
            obj.alpha = A(4,:);
        end
        
        function q = fkm(obj,theta, ith)
            %   dq = fkm(theta) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   end-effector pose.
            %   theta is the vector of joint variables
            
            if nargin == 3
                n = ith;               
            else
                n = obj.links;
            end
            
            if length(theta) ~= (obj.links - obj.n_dummy)
                error('Incorrect number of joint variables');
            end
            
            q=DQ(1);
            
            j = 0;
            for i=1:n
                if obj.dummy(i) == 1
                    q = q*dh2dq(obj,0,i);
                    j = j + 1;
                else
                    q = q*dh2dq(obj,theta(i-j),i);
                end
            end
        end
        
        function dq = dh2dq(obj,theta,i)
            %   For a given link's DH parameters, calculate the correspondent dual
            %   quaternion
            %   Usage: dq = dh2dq(theta,i), where
            %          theta: joint angle
            %          i: link number
            
            if nargin ~= 3
                error('Wrong number of arguments. The parameters are theta and the correspondent link')
            end
            d = obj.d(i);
            a = obj.a(i);
            alpha = obj.alpha(i);
            
            if strcmp(obj.convention,'standard')
                q(1)=cos((theta+obj.theta(i))/2)*cos(alpha/2);
                q(2)=cos((theta+obj.theta(i))/2)*sin(alpha/2);
                q(3)=sin((theta+obj.theta(i))/2)*sin(alpha/2);
                q(4)=sin((theta+obj.theta(i))/2)*cos(alpha/2);
                d2=d/2;
                a2=a/2;
                
                
                q(5)=-d2*q(4)-a2*q(2);
                q(6)=-d2*q(3)+a2*q(1);
                q(7)=d2*q(2)+a2*q(4);
                q(8)=d2*q(1)-a2*q(3);
            else
                h1 = cos((theta+obj.theta(i))/2)*cos(alpha/2);
                h2 = cos((theta+obj.theta(i))/2)*sin(alpha/2);
                h3 = sin((theta+obj.theta(i))/2)*sin(alpha/2);
                h4 = sin((theta+obj.theta(i))/2)*cos(alpha/2);
                q(1)= h1;
                q(2)= h2;
                q(3)= -h3;
                q(4)= h4;
                d2=d/2;
                a2=a/2;
                
                
                q(5)=-d2*h4-a2*h2;
                q(6)=-d2*h3+a2*h1;
                q(7)=-(d2*h2+a2*h4);
                q(8)=d2*h1-a2*h3;
            end
            
            dq=DQ(q);
        end
        
        function p = get_p(obj,q)
            p(1) = 0;
            p(2)=q(2)*q(4) + q(1)*q(3);
            p(3)=q(3)*q(4) - q(1)* q(2);
            p(4)=(q(4)^2-q(3)^2-q(2)^2+q(1)^2)/2;
            p(5)=0;
            p(6)=q(2)*q(8)+q(6)*q(4)+q(1)*q(7)+q(5)*q(3);
            p(7)=q(3)*q(8)+q(7)*q(4)-q(1)*q(6)-q(5)*q(2);
            p(8)=q(4)*q(8)-q(3)*q(7)-q(2)*q(6)+q(1)*q(5);
        end
        
        
        
        function J = jacobian(obj,theta)
            % J = jacobian(theta) returns the dual quaternion Jacobian, where
            % theta is the vector of joint variables
            q_effector = obj.fkm(theta);
            
            q = DQ(1);
            J= zeros(8,obj.links-obj.n_dummy);
            ith=0;
            
            for i = 0:obj.links-1
                %If the joint is not dummy, there is a correspondent
                if ~obj.dummy(i+1)
                    %Use the standard DH convention
                    if strcmp(obj.convention,'standard')
                        p = obj.get_p(q.q);
                    else %Use the modified DH convention
                        w=DQ([0,0,-sin(obj.alpha(i+1)),cos(obj.alpha(i+1)),0,0,-obj.a(i+1)*cos(obj.alpha(i+1)),-obj.a(i+1)*sin(obj.alpha(i+1)) ] );
                        p =0.5*q*w*q';
                    end
                    q = q*obj.dh2dq(theta(i+1),i+1);
                    j = p * q_effector;
                    J(:,ith+1) = j.q;
                    ith = ith+1;
                end
            end
        end
        
    end
    
    methods(Static)
        
        function Jp = jacobp(J,x)
            %Given the dual quaternion Jacobian J and the corresponding
            %dual quaternion, Jp = jacobp(J,x) returns the translation Jacobian; that it,
            %the Jacobian that satisfies the relation dot_p = Jp * dot_theta,
            %where dot_p is the time derivative of the
            %translation quaternion and dot_theta is the time derivative of
            %the joint vector.
            x = DQ(x);
            Jp = 2*haminus4(x.P')*J(5:8,:)+2*hamiplus4(x.D)*DQ_kinematics.C4*J(1:4,:);
        end
        
        function Jd = jacobd(J,x)
            %Given the dual quaternion Jacobian J and the corresponding
            %dual quaternion, Jd = jacobd(J,x) returns the distance Jacobian; that it,
            %the Jacobian that satisfies the relation dot_(d^2) = Jd * dot_theta,
            %where dot_(d^2) is the time derivative of the
            %square of the distance between the end-effector and the base and dot_theta is the time derivative of
            %the joint vector.
            x = DQ(x);
            p = translation(x);
            Jp = DQ_kinematics.jacobp(J,x);
            Jd=2*vec4(p)'*Jp;
        end
    end
end