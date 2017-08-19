% CLASS DQ_KinematicController implement several (first order) kinematic controllers 
% for a broad class of robots (i.e., controllers based on the model vec8(xdot) = J * qdot)
%   
% Ways of defining a new controller:
%   controller = DQ_KinematicController(robot), where robot is an object of
%   type DQ_kinematics
%
% Controllers (for more information, type help DQ_KinematicController.<CONTROLLER_NAME>):
%       pseudoinverse_pose_controller
%       damped_pseudoinverse_pose_controller
%       parsimonious_pose_controller
%
% All controllers are based on the following data structure:
%           Minimal structure (i.e., all controllers must have the following
%           information):
%               controller_variables.q: current robot configuration;
%               controller_variables.K: current gain matrix;
%               controller_variables.xd: current desired end-effector pose;
% Specific controllers have specific fields in addition to the minimal
% structure. Type help DQ_KinematicController.<CONTROLLER_NAME>) for more
% details.
%
% See also DQ_kinematics
%          DQ_KinematicController.pseudoinverse_pose_controller
%          DQ_KinematicController.damped_pseudoinverse_pose_controller
%          DQ_KinematicController.parsimonious_pose_controller
%          DQ_KinematicController.left_invariant_pseudoinverse_pose_controller

% (C) Copyright 2017 DQ Robotics Developers
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
classdef DQ_KinematicController
% Information for developers
% We use some standard definitions for variables:
% u : control input
% J : Jacobian matrix
% x : current end-effector pose
% xd : desired end-effector pose
% q : current robot configuration
% qd : desired robot configuration
% e : error

    properties
        robot; % DQ_kinematics object     
    end
    
    methods
        function obj = DQ_KinematicController(robot)
            if nargin ~= 1
                error_message = ['There is no associated robot. Please use the' ...
                                'following syntax controller = DQ_KinematicController(robot),'...
                                'where robot is an object of type DQ_kinematics'];
                error(error_message);
            else
                obj.robot = robot;
            end
        end
        
        function [u, varargout] = pseudoinverse_pose_controller(obj,controller_variables) 
        % pseudoinverse_pose_controller    Classic pose controller based on the 
        % pseudoinverse of the Jacobian matrix and the Euclidean error: 
        % u = pinv(J)*K*vec8(xd-x).
        %
        % [u,e] = pseudoinverse_pose_controller(controller_variables)
        % returns the control input u (joint velocities) and the current error
        % e = vec(xd - x), where xd is the desired pose and x is the current one.
        %
        % u = = pseudoinverse_pose_controller(controller_variables) returns
        % only the control input u.
        %
        % This controller needs only the minimal structure for controller
        % variables (see help DQ_KinematicController)
        
            J = obj.robot.jacobian(controller_variables.q);
            x = obj.robot.fkm(controller_variables.q);
            e = vec8(controller_variables.xd - x);
            u = pinv(J)*controller_variables.K*e; 
            if nargout == 2
                varargout{1} = e;
            end
        end
        
        function [u, varargout] = damped_pseudoinverse_pose_controller(obj,controller_variables)
        % damped_pseudoinverse_pose_controller    Pose controller based on the 
        % damped pseudoinverse of the Jacobian matrix and the Euclidean error: 
        % u = dinv(J)*K*vec8(xd-x), where 
        % dinv(J) = J'*inv(J*J'+lambda^2*I), if n > 6 (redundant system)
        % dinv(J) = inv(J'*J + lambda^2*I)*J' if n <= 6 (underactuated or completely actuated)
        % 
        % [u,e] = damped_pseudoinverse_pose_controller(controller_variables)
        % returns the control input u (joint velocities) and the current error
        % e = vec(xd - x), where xd is the desired pose and x is the current one.
        %
        % u = = pseudoinverse_pose_controller(controller_variables) returns
        % only the control input u.
        %
        % This controller needs the minimal structure for controller
        % variables (see help DQ_KinematicController) plus the damping
        % factor.
            J = obj.robot.jacobian(controller_variables.q);
            x = obj.robot.fkm(controller_variables.q);
            e = vec8(controller_variables.xd - x);
            
            %Calculates the damped pseudoinverse
            dof = length(obj.robot.q);
            if dof <= 6
                dinv_J = (J'*J + controller_variables.lambda^2*eye(dof))\J';
            else
                dinv_J = J'/(J*J'+controller_variables.lambda^2*eye(8));
            end
                
            u = dinv_J*controller_variables.K*e;
            if nargout == 2
                varargout{1} = e;
            end
        end
        
        function [u, varargout] = parsimonious_pose_controller(obj,controller_variables) 
        % parsimonious_pose_controller    Parsimonious pose controller based on the 
        % paper 
        % [1] V. M. Goncalves, P. Fraisse, A. Crosnier, and B. V. Adorno, "Parsimonious Kinematic Control of 
        % Highly Redundant Robots," IEEE Robot. Autom. Lett., vol. 1, no. 1, pp. 65?72, Jan. 2016.
        %
        % [u,e,fval] = parsimonious_pose_controller(controller_variables)
        % returns the control input u (joint velocities), the current error
        % e = vec(xd - x), where xd is the desired pose and x is the
        % current one, and fval is the value of the objective function at
        % u.
        %
        % [u,e] = parsimonious_pose_controller(controller_variables)
        % returns the control input u (joint velocities) and the current error
        % e = vec(xd - x), where xd is the desired pose and x is the current one.
        %
        % u = parsimonious_pose_controller(controller_variables) returns
        % only the control input u.
        %
        % This controller needs the minimal structure for controller
        % variables (see help DQ_KinematicController) plus the additional equality and
        % inequality constraints.
        % TODO: This is just the basic implementation, so for the moment
        % there is no possibility of adding additional inequality or
        % equality constraints.
        % The optimization problem is slightly different from the one of
        % [1], in the sense that the convergence parameter (eta) is implemented
        % here as a positive definite matrix, not just a scalar. 
        
            % DOFS
            ROBOT_DOFS   = length(obj.robot.q);
            % The numbers of the next variable are related to Problems 6 and 7 of
            % Gon?alves et al. (2016). More specifically, g = [qp_dot, qn_dot, y, za,
            % zc], where q_dot = qp_dot - qn_dot, y comes from Problem 6 and za (it has 
            % the same dimension of y) and zc are the slack variables.
            PROBLEM_DOFS = ROBOT_DOFS*2+8*2+1; 
             % Ones
            ones_rdof    = ones(ROBOT_DOFS,1); % ones vector related to the joint space
            ones_e       = ones(8,1); % ones vector related to the task space
            % Zeros
            zeros_rdof   = zeros(ROBOT_DOFS,1); % zeroes vector related to the joint space (Do not confuse this with a slack variable)
            zeros_e      = zeros(8,1); % zeroes vector related to the task space
            zeros_p      = zeros(PROBLEM_DOFS,1); % zeroes vector related to the problem
            % Identity matrices
            I_rdof  = eye(ROBOT_DOFS); 
            I_e     = eye(8); 
            I_p     = eye(PROBLEM_DOFS); 
       
        
            J = obj.robot.jacobian(controller_variables.q);
            x = obj.robot.fkm(controller_variables.q);
            % The error is inverted with respect to the classic pseudoinverse
            % controller. Of course, we could change the sign of the
            % classic pseudoinverse error and also change the sign of the
            % control input as well in that case. 
            e = vec8(x - controller_variables.xd);
            
            %% Linear programming definitions. See [1-2]
            f   = [-ones_e'*J ones_e'*J 2*ones_e' zeros_e' 0];     % [-1'*J 1'*J 2*1' 0' 0];
            % A and b are used to enforce the constraint g >=0 (Problem 7 of Gon?alves et al., 2016)
            A   = [-I_p];
            b   = [zeros_p];
            % Aeq and Beq are used to enforce the equality constraint of Problem 7
            % of Gon?alves et al., 2016.
            Aeq = [[J;ones_rdof'], [-J;ones_rdof'], [-I_e;zeros_e'] , [I_e;zeros_e'] , [zeros_e;1]];   % [J;1' -J;1' -I;0' I;0' 0;1]
            beq = [-controller_variables.K*e;controller_variables.beta*norm(e,1)];
    
            %% Linear Programming    
            % The syntax of linprog has changed starting from version 9.1. Thus we 
            % write code to both recent and older versions in order to guarantee
            % backward compability.    
            if verLessThan('matlab','8.5')        
                options = optimoptions('linprog','Algorithm','dual-simplex','Display','off');
                [g,fval] = linprog(f,A,b,Aeq,beq,[],[],[],options);      
            else
            % Linear Programming (The default algorithm in matlab is the dual-simplex)
                options = optimoptions('linprog','Algorithm','dual-simplex');
                g = linprog(f,[],[],Aeq,beq,zeros(PROBLEM_DOFS,1),[],options); % g = [QP' QN' Y ZB ZC]
            end
            QP = g(1:ROBOT_DOFS);
            QN = g(ROBOT_DOFS+1:ROBOT_DOFS*2);
            u = QP-QN; % u = QP-QN
            
            if nargout >= 2
                varargout{1} = e;
                if nargout == 3 % Returns the value of the objective function (See Eq. (7) of [1]).                      
                    varargout{2} = fval - ones_e'*controller_variables.K * e;
                end
            end            
        end
        
        function [u, varargout] = left_invariant_pseudoinverse_pose_controller(obj,controller_variables) 
        % Pose controller based on the left invariant error (1-x'*xd) and the
        % pseudoinverse of the projected Jacobian matrix J. More specifically,  
        % u = pinv(N)*K*vec8(1-x'*xd), where N = haminus8(xd)*DQ.C8*J, with 
        % K being a positive definite gain matrix, x is the current end-effector
        % pose, xd is the desired end-effector pose, and J is the robot
        % Jacobian matrix.
        %
        % [u,e] = left_invariant_pseudoinverse_pose_controller(controller_variables)
        % returns the control input u (joint velocities) and the current error
        % e = vec8(1-x'*xd), where xd is the desired pose and x is the current one.
        %
        % u = = pseudoinverse_pose_controller(controller_variables) returns
        % only the control input u.
        %
        % This controller needs only the minimal structure for controller
        % variables (see help DQ_KinematicController)
        
            J = obj.robot.jacobian(controller_variables.q);
            x = obj.robot.fkm(controller_variables.q);
            N = haminus8(controller_variables.xd)*DQ.C8*J;
            e = vec8(1 - x'*controller_variables.xd);
            u = pinv(N)*controller_variables.K*e; 
            if nargout == 2
                varargout{1} = e;
            end
        end 
    end
end

