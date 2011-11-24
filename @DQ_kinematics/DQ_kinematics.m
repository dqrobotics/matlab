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
%       


classdef DQ_kinematics
    properties
        links;
        theta,d,a,alpha;
        convention
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

            obj.links = size(A,2);

            for i = 1:obj.links
                obj.theta(i)=A(1,i);
                obj.d(i)=A(2,i);
                obj.a(i)=A(3,i);
                obj.alpha(i)=A(4,i);
            end
        end
              
        function q = fkm(obj,theta)
            %   dq = fkm(theta) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   end-effector pose.
            %   theta is the vector of joint variables
            if length(theta) ~= obj.links
                error('Incorrect number of joint variables');
            end
            q=DQ(1);
            for i=1:obj.links
                q = q*dh2dq(obj,theta(i),i);
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
                q(1)=cos(theta/2)*cos(alpha/2);
                q(2)=cos(theta/2)*sin(alpha/2);
                q(3)=sin(theta/2)*sin(alpha/2);
                q(4)=sin(theta/2)*cos(alpha/2);
                d2=d/2;
                a2=a/2;


                q(5)=-d2*q(4)-a2*q(2);
                q(6)=-d2*q(3)+a2*q(1);
                q(7)=d2*q(2)+a2*q(4);
                q(8)=d2*q(1)-a2*q(3);
            else

                q(1)=cos(theta/2)*cos(alpha/2);
                q(2)=cos(theta/2)*sin(alpha/2);
                q(3)=-sin(theta/2)*sin(alpha/2);
                q(4)=sin(theta/2)*cos(alpha/2);
                d2=d/2;
                a2=a/2;


                q(5)=-d2*q(4)-a2*q(2);
                q(6)=-d2*q(3)+a2*q(1);
                q(7)=-(d2*q(2)+a2*q(4));
                q(8)=d2*q(1)-a2*q(3);
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
            J= zeros(8,obj.links);

            for i = 0:obj.links-1
                %Use the standard DH convention
                if strcmp(obj.convention,'standard')
                    p = obj.get_p(q.q);
                else %Use the modified DH convention
                    w=DQ([0,0,-sin(obj.alpha(i+1)),cos(obj.alpha(i+1)),0,0,-obj.a(i+1)*cos(obj.alpha(i+1)),-obj.a(i+1)*sin(obj.alpha(i+1)) ] );                    
                    p =0.5*q*w*q';
                end
                q = q*obj.dh2dq(theta(i+1),i+1);
                j = p * q_effector;
                J(:,i+1) = j.q;
            end
        end
    end
end