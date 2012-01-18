% CLASS DQ_cdts
% Usage: cdts = DQ_cdts(robot1,base1,robot2,base2), where:
% - robot1 and robot2 are objects of type DQ_kinematics;
% - base1 and base2 are dual quaternions representing the pose of each arm
%   with respect to the world.
%
% By using this class, the cooperative system is described by the
% cooperative variables xa and xr, and their respective Jacobians Ja and Jr
%
% Type DQ_cdts.(method or property) for specific help.
% Ex.: help DQ_cdts.xa
%
% METHODS:
%       xa
%       xr
%       Ja
%       Jr
%       x1
%       x2
%


classdef DQ_cdts
    properties
        robot1, robot2, base1, base2;
    end
    
    methods
        function obj = DQ_cdts(robotA,baseA , robotB, baseB)
            if ~isa(robotA,'DQ_kinematics') || ~isa(robotB,'DQ_kinematics')
                error('The DQ_cdts class must be initialized with the kinematics information of each robot');
            end
            obj.robot1 = robotA;
            obj.robot2 = robotB;
            obj.base1 = DQ(baseA);
            obj.base2 = DQ(baseB);
        end
        
        function x = x1(obj,theta)
            % x = x1(theta) returns the dual position of the first arm,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            theta1=theta(1:obj.robot1.links);
            x = obj.base1*obj.robot1.fkm(theta1);
        end
        
        function x = x2(obj,theta)
            % x = x2(theta) returns the dual position of the second arm,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            theta2=theta(obj.robot1.links+1:end);
            x = obj.base2*obj.robot2.fkm(theta2);
        end
        
        function x = xr(obj,theta)
            % x = xr(theta) returns the relative dual position,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            theta1=theta(1:obj.robot1.links);
            theta2=theta(obj.robot1.links+1:end);
            
            x1 = obj.base1*obj.robot1.fkm(theta1);
            x2 = obj.base2*obj.robot2.fkm(theta2);
            
            x = x2'*x1;
        end
        
        function x = xa(obj,theta)
            % x = xr(theta) returns the absolute dual position,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            theta2=theta(obj.robot1.links+1:end);
            x2 = obj.base2*obj.robot2.fkm(theta2);
            xr = obj.xr(theta);
            x = x2*(xr^0.5);
        end
        
        function jac = Jr(obj,theta)
            % x = xr(theta) returns the relative Jacobian,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            theta1=theta(1:obj.robot1.links);
            theta2=theta(obj.robot1.links+1:end);
            
            x1 = obj.base1*obj.robot1.fkm(theta1);
            x2 = obj.base2*obj.robot2.fkm(theta2);
            
            jacob1 = hamiplus8(obj.base1)*obj.robot1.jacobian(theta1);
            jacob2 = hamiplus8(obj.base2)*obj.robot2.jacobian(theta2);
            
            jac = [hamiplus8(x2')*jacob1, haminus8(x1)*DQ_kinematics.C8*jacob2];
        end
        
        function jac = Ja(obj, theta)
            % x = xr(theta) returns the absolute Jacobian,
            % where theta is the joint position of the resultant system;
            % that is, theta = [theta1;theta2]
            theta1=theta(1:obj.robot1.links);
            theta2=theta(obj.robot1.links+1:end);
            
            x1 = obj.base1*obj.robot1.fkm(theta1);
            x2 = obj.base2*obj.robot2.fkm(theta2);
            
            jacob2 = hamiplus8(obj.base2)*obj.robot2.jacobian(theta2);
            
            jacobr = obj.Jr(theta);
            xr=x2'*x1;
            
            jacob_r2 = 0.5*haminus4((xr.P')^0.5)*jacobr(1:4,:);
            jacobp_r = DQ_kinematics.jacobp(jacobr,xr);
            
            jacob_xr_2 = [jacob_r2; 0.25*(haminus4(xr.P^0.5)*jacobp_r+hamiplus4(translation(xr))*jacob_r2)];
            
            jac = haminus8(xr^0.5)*[zeros(8,length(theta1)),jacob2]+hamiplus8(x2)*jacob_xr_2;
        end
    end
end