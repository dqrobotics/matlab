% DQ_WholeBody Robots composed of multiple kinematic chains
%
% Usage: robot = DQ_WholeBody(first_chain_element), where
% first_chain_element is an object of DQ_Kinematics or one of its
% subclasses.
%
% Alternatively, robot = DQ_WholeBody(first_chain_element,'reversed'),
% where the optional argument 'reversed' indicates that the kinematic chain
% is reversed.
%
% DQ_WholeBody Properties:
%       chain - Contains all elements in the serial kinematic chain.
%       reversed - Vector with the same dimension of chain to indicate if the
%                corresponding kinematic chain is reversed.
%       dim_configuration_space - Dimension of the whole-body configuration
%                               space
%
% DQ_WholeBody Methods:
%       add - Adds a new element to the end of the serially coupled
%             kinematic chain.
%       add_reversed - Adds a new element, but in reverse order, to the end
%                      of the serially coupled kinematic chain.
%       fkm - Returns the forward kinematic model of the whole-body chain.
%       get_chain - Returns the complete kinematic chain.
%       get_dim_configuration_space - Returns the dimension of the whole-body
%                                      configuration space.
%       plot - Draws the whole kinematic chain.
%       pose_jacobian - Returns the whole-body pose Jacobian.
%
%       raw_fkm - Analogous to FKM, but without considering base and
%                 end-effector changes.
%       sequential - Reorganize a sequential configuration vector in the
%                    ordering required by each kinematic chain (i.e.,
%                    the vector blocks corresponding to reversed chains are
%                    reversed).
%
% For a complete list of methods, including the one from the super classes,
% type 'doc DQ_WholeBody'
%
% See also DQ_kinematics, DQ_MobileBase

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

classdef DQ_WholeBody < DQ_Kinematics
    properties (Access = protected)
        
        % Contains all elements in the serial kinematic chain. Usually they
        % are objects from DQ_Kinematics or one of its subclasses. They can also
        % be DQ elements that represent constant rigid transformations.
        chain;
        
        % Logical vector with the same dimension of the 'chain' vector. Given
        % chain{ith}, if reversed(ith) == 'true', then the corresponding
        % fkm of chain{ith} is its conjugate and the corresponding pose_jacobian is
        % DQ.C8*J.
        reversed;
        dim_configuration_space;
    end
    
    methods
        function obj = DQ_WholeBody(varargin)
            robot = varargin{1};
            if ~isa(robot,'DQ_Kinematics')
                error(['The first argument must be a DQ_Kinematics '...
                    'object or one of its subclasses.']);
            end
            obj.chain{1} = robot;
            obj.dim_configuration_space = robot.get_dim_configuration_space();
            % The default behavior is to have direct chains (i.e., not reversed)
            if nargin == 1
                obj.reversed(1) = false;
            elseif nargin == 2 && strcmp(varargin{2},'reversed')
                obj.reversed(1) = true;
            else
                error_mesg = sprintf(['Invalid number of parameters. Usage:\n'...
                    'robot = DQ_WholeBody(first_chain_element)\n'...
                    'robot = DQ_WholeBody(first_chain_element,''reversed'')']);
                error(error_mesg);
            end
            
        end
        
        function add(obj, new_chain)
            % Adds a new element at the end of the kinematic chain.
            %
            % ADD(new_chain) adds a new DQ_Kinematics or DQ object to the end
            % of the serial kinematic chain. If new_chain is
            % a DQ_Kinematics object, the end-effector of the previous chain is
            % connected to the base of the new chain. If it is a DQ object,
            % then the constant transformation, which represents a constant
            % rigid motion, is just added to the chain in an analogous manner.
            % TODO: make it possible to add the kinematic chain in any
            %       position of the chain.
            %       3) make it possible to create branch structures
            len = length(obj.chain);
            obj.chain{len + 1} = new_chain;
            % The dimension of the configuration space increases only if
            % the the new element in the chain is not a constant rigid
            % transformation
            if isa(new_chain,'DQ_Kinematics')
                obj.dim_configuration_space = obj.dim_configuration_space + ...
                    new_chain.get_dim_configuration_space();
            elseif ~isa(new_chain,'DQ')
                error(['Only DQ_Kinematics and DQ objects can be added to the'...
                    ' chain']);
            end
            obj.reversed(len + 1) = false;
        end
        
        function add_reversed(obj, robot)
            % Add a new reversed element to the end of the kinematic chain.
            %
            % ADD_REVERSED(robot) adds a new robot to the end of the serial
            % kinematic chain, but its kinematic chain is reversed.
            % More specifically, the end-effector of the previous chain is
            % connected to the end-effector of the new chain.
            % TODO: 1) make it possible to add the kinematic chain in any
            %       position of the chain.
            %       2) make it possible to create branch structures
            if isa(new_chain,'DQ_Kinematics')
                len = length(obj.chain);
                obj.chain{len + 1} = robot;
                obj.dim_configuration_space = obj.dim_configuration_space + ...
                    robot.get_dim_configuration_space();
                obj.reversed(len + 1) = true;
            else
                error('Only DQ_Kinematics objects can be added in reverse mode');
            end
        end
        
        function x = fkm(obj,q,ith, jth)
            % Returns the forward kinematic model of the whole-body chain.
            %
            % x = FKM(q) receives the configuration vector q of the whole
            % kinematic chain and returns the pose of the last frame.
            % x = FKM(q, ith) calculates the forward kinematics up to the ith
            % kinematic chain.
            % x = RAW_FKM(q, ith, jth) calculates the forward kinematics up to
            % the jth link of the ith kinematic chain.
            % FKM takes into account the reference frame.
            if nargin > 4
                error('Invalid number of arguments');
            elseif nargin == 4 
                x = obj.reference_frame * raw_fkm(obj,q,ith,jth);
            elseif nargin == 3
                x = obj.reference_frame * raw_fkm(obj,q,ith);
            else
                x = obj.reference_frame * raw_fkm(obj,q);
            end
        end
        
        function ret = get_chain(obj,ith)
            % Returns the complete kinematic chain.
            if nargin > 2
                error('Invalid number of parameters');
            elseif nargin == 2
                ret = obj.chain{ith};
            else
                ret = obj.chain;
            end
        end
        
        function ret = get_dim_configuration_space(obj)
            % Returns the dimension of the whole-body configuration space.
            ret = obj.dim_configuration_space;
        end
        
        function plot(obj,q)
            % Draws the whole kinematic chain.
            %
            % PLOT(q) draws the whole kinematic chain, given 'q'. It does
            % all necessary transformations to take into account reverse
            % chains and direct kinematic chains.
            
            if isa(obj.chain{1}, 'DQ_Kinematics')
                dim_conf_space = obj.chain{1}.get_dim_configuration_space();
                %DQ_MobileBase does not have 'nojoints' property
                if isa(obj.chain{1}, 'DQ_MobileBase')
                    plot(obj.chain{1},q(1:dim_conf_space));
                elseif isa(obj.chain{1}, 'DQ_SerialManipulator')
                    % To improve plot performance, specially for very large
                    % configuration space dimensions, we never plot the
                    % joints of serial manipulators.
                    
                    % If the first kinematic chain is a fixed-base serial
                    % chain *and* reversed, we must adapt its base frame so
                    % that the DQ_Kinematics/plot function, which always
                    % start ploting from its base frame, plots the serial
                    % chain with the end-effector coinciding with the
                    % whole-body base frame. (Note that each individual chain
                    % has its own base frame used to determine its spatial
                    % location).
                    if obj.reversed(1)
                        current_base_frame = obj.get_base_frame() * ...
                            obj.raw_fkm(q,1);
                    else
                        % if the chain is not reversed, then its base frame
                        % must coincide with the whole-body base frame.
                        current_base_frame = obj.get_base_frame();
                    end
                    obj.chain{1}.set_base_frame(current_base_frame);
                    plot(obj.chain{1},q(1:dim_conf_space),'nojoints');
                end
                j = dim_conf_space + 1;
            else
                j = 1;
            end
            
            % Iterate over the chain
            for i = 2:length(obj.chain)
                % If the first element in the kinematic chain is a mobile
                % base, its fkm coincides with the base location, already
                % considering a frame displacement, if aplicable (e.g., in case
                % the mobile base frame is not in its center).
                if isa(obj.chain{1}, 'DQ_MobileBase')
                    current_base_frame = obj.fkm(q,i-1);
                else
                    % The first element in the chain has a fixed-base robot,
                    % which may be located arbitrarily in the workspace with a
                    % rigid transformation given by base_frame (which is not
                    % necessarily the same as the reference frame).
                    current_base_frame = obj.get_base_frame() * ...
                        obj.raw_fkm(q,i-1);
                end
                
                % Constant rigid transformations do not change the
                % dimension of the configuration space. Furthermore, we do
                % not plot them (this behavior may be changed in the future,
                % though).
                if isa(obj.chain{i}, 'DQ_Kinematics')
                    obj.chain{i}.set_base_frame(current_base_frame);
                    
                    dim = obj.chain{i}.get_dim_configuration_space();
                    qi = q(j : j + dim - 1);
                    j = j + dim;
                    
                    % We do not plot names, bases, and coordinate systems of
                    % the intermediate kinematic chains.
                    if i < length(obj.chain)
                        plot(obj.chain{i},qi, 'nobase', ...
                            'nowrist', 'noname', 'nojoints');
                    else
                        % But we plot the coordinate system of the whole-body
                        % end-effector.
                        plot(obj.chain{i},qi, 'nobase', 'noname', 'nojoints');
                    end
                end
            end
            
        end
        
        function J = pose_jacobian(obj,q,ith,jth)
            % Returns the whole-body pose Jacobian.
            %
            % J = POSE_JACOBIAN(q) receives the configuration vector q of the whole
            % kinematic chain and returns the jacobian matrix J that satisfies
            % vec8(xdot) = J * q_dot, where q_dot is the configuration velocity
            % and xdot is the time derivative of the unit dual quaternion that
            % represents the end-effector pose.
            % J = POSE_JACOBIAN(q, ith) calculates the Jacobian up to the ith
            % kinematic chain.
            % J = POSE_JACOBIAN(q, ith,jth) calculates the Jacobian up to the 
            % jth link of the ith kinematic chain.
            % 
            % For more details, see Eq. (4.7) of ?B. V. Adorno,
            % ?Two-arm Manipulation: From Manipulators to Enhanced
            % Human-Robot Collaboration [Contribution ? la manipulation ?
            % deux bras : des manipulateurs ? la collaboration
            % homme-robot],? Universit? Montpellier 2, 2011.
            
            partial_chain = false;
            if nargin > 4
                error('Invalid number of parameters')
            elseif nargin == 4
                % find the Jacobian up to the jth link of the ith
                % intermediate kinematic chain
                partial_chain = true;
                n = ith;            
            elseif nargin == 3
                % find the Jacobian up to the ith intermediate kinematic
                % chain
                n = ith;
            else
                % find the jacobian of the whole chain
                n = length(obj.chain);
            end
            
            if partial_chain == true                
                x_0_to_n = obj.fkm(q,n,jth);
            else
                x_0_to_n = obj.fkm(q,n);
            end
            
            j = 1;
            
            for i = 0:n-1
                if partial_chain == true && i == n-1
                    x_0_to_iplus1 = obj.fkm(q,i+1,jth);
                else                    
                    x_0_to_iplus1 = obj.fkm(q,i+1);
                end
                x_iplus1_to_n = x_0_to_iplus1'*x_0_to_n;
                
                % Constant rigid transformations in the chain do not change the
                % dimension of the configuration space.
                if isa(obj.chain{i+1}, 'DQ_Kinematics')
                    dim = obj.chain{i+1}.get_dim_configuration_space();
                    q_iplus1 = q(j : j + dim - 1);
                    j = j + dim;
                    
                    % TODO: The code below can be cleaned up to prevent
                    % duplication.
                    if obj.reversed(i+1) == true
                        if partial_chain == true && i == n-1                            
                            A = haminus8(obj.chain{i+1}.fkm(q_iplus1,dim-jth)) * ...
                                DQ.C8 * obj.chain{i+1}.pose_jacobian(q_iplus1);
                            B =  hamiplus8(obj.chain{i+1}.fkm(q_iplus1)) * ...
                                 [obj.chain{i+1}.pose_jacobian(q_iplus1,dim-jth),zeros(8,jth)];                           
                           
                            reverse_pose_jacobian_jth =  A + B;
                        
                            L{i+1} = hamiplus8(obj.fkm(q,i)) * ...
                                        haminus8(x_iplus1_to_n) * ...
                                            reverse_pose_jacobian_jth;
                                    
                        else
                            L{i+1} = hamiplus8(obj.fkm(q,i)) * ...
                                haminus8(x_iplus1_to_n) * DQ.C8 * ...
                                    obj.chain{i+1}.pose_jacobian(q_iplus1);
                        end
                    else
                        if partial_chain == true && i == n-1
                            L{i+1} = hamiplus8(obj.fkm(q,i)) * ...
                                haminus8(x_iplus1_to_n) * ...
                                    obj.chain{i+1}.pose_jacobian(q_iplus1,jth);
                        else
                            L{i+1} = hamiplus8(obj.fkm(q,i)) * ...
                                        haminus8(x_iplus1_to_n) * ...
                                        obj.chain{i+1}.pose_jacobian(q_iplus1);
                        end
                    end
                end
            end
            J = cell2mat(L);
        end
        
        
        
        
        function x = raw_fkm(obj, q, ith, jth)
        % Analogous to FKM, but without considering base and end-effector changes.
        %
        % x = RAW_FKM(q) receives the configuration vector q of the whole
        % kinematic chain and returns the pose of the last frame.
        % x = RAW_FKM(q, ith) calculates the forward kinematics up to the ith
        % kinematic chain.
        % x = RAW_FKM(q, ith, jth) calculates the forward kinematics up to
        % the jth link of the ith kinematic chain.
        % RAW_FKM does not take into account the reference frame.
            
            % By default, the fkm is taken up to the end of the ith
            % kinematic chain.
            partial_chain = false;
            if nargin > 4
                error('Invalid number of arguments');
            elseif nargin == 4
                % the forward kinematics is calculated up to the jth link
                % of the ith kinematic chain.
                n = ith;
                partial_chain = true;
            elseif nargin == 3
                % the forward kinematics is calculated up to the ith
                % kinematic chain
                n = ith;
            else
                % the whole-body kinematic chain is taken into consideration
                n = length(obj.chain);
            end
            
            x = DQ(1);
            j = 1; % first configuration vector (q1)
            
            % Iterate over the chain
            for i = 1:n
                % TODO: The next three lines shouldn't cost much, but this
                % implementation can be improved. For instance, we can
                % store the size of each configuration vector whenever we
                % add a new robot into the serial kinematic chain.
                if isa(obj.chain{i}, 'DQ_Kinematics')
                    dim = obj.chain{i}.get_dim_configuration_space();
                    qi = q(j : j + dim - 1);
                    j = j + dim;
                end
                
                if obj.reversed(i) == true
                    % The chain is reversed
                    if partial_chain == true && i == n
                        x_base_to_effector = obj.chain{i}.fkm(qi);
                        kth = dim - jth;
                        x_base_to_kth = obj.chain{i}.fkm(qi,kth);
                        
                        x_effector_to_kth = x_base_to_effector'*x_base_to_kth;
                        x = x * x_effector_to_kth;
                    else
                        x = x*obj.chain{i}.fkm(qi)';
                    end
                elseif isa(obj.chain{i}, 'DQ')
                    % Is it a rigid transformation? (Rigid transformations are
                    % never reversed in the chain because a reverse rigid
                    % transformation is accomplished by using its
                    % conjugate when adding it to the chain.)
                    x = x*obj.chain{i};
                else
                    % It's neither a rigid transformation nor a reversed
                    % chain; that is, it's just a regular one.
                    if partial_chain == true && i == n
                        x = x*obj.chain{i}.fkm(qi,jth);
                    else                        
                        x = x*obj.chain{i}.fkm(qi);
                    end
                end
            end
        end
        
        function q = sequential(obj,q)
        % Reorganize a sequential configuration vector in the required ordering.
        %
        % SEQUENTIAL(q) takes the configuration vector q in a sequential
        % ordering and automatically reorganize it in the order determined
        % by each kinematic chain. For instance, given a reverse chain
        % composed of six joints, joint 6 is the first one and joint 1 is
        % the sixth one, opposed to the commonly used forward chain, where
        % joint 1 is the first joint and joint 6 is sixth one.
            j = 1;
            for i = 1:length(obj.chain)
                if isa(obj.chain{i}, 'DQ_Kinematics')
                    dim = obj.chain{i}.get_dim_configuration_space();
                    qi = q(j : j + dim - 1);
                    if ~obj.reversed(i)
                        q(j : j + dim - 1) = qi;
                        
                    else
                        q(j : j + dim - 1) = flip(qi);
                    end
                    j = j + dim;
                end
            end
        end
        
        
    end
end