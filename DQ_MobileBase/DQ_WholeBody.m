classdef DQ_WholeBody < handle
    properties %(Access = protected)
        chain;
        dim_configuration_space;
    end
    
    methods
        function obj = DQ_WholeBody(robot)
            % TODO: test if robot is an instance of DQ_Kinematics
            obj.chain{1} = robot;
            obj.dim_configuration_space = robot.get_dim_configuration_space();
        end
        
        function ret = get_dim_configuration_space(obj)
            ret = obj.dim_configuration_space;
        end
        
        function add(obj, robot)
            % add(robot) adds a robot to the end of the serial kinematic chain
            % TODO: 1) test if robot is an instance of DQ_Kinematics
            %       2) make it possible to add the kinematic chain in any
            %       position of the chain. 
            %       3) make it possible to create branch structures
            len = length(obj.chain);
            obj.chain{len + 1} = robot;
            obj.dim_configuration_space = obj.dim_configuration_space + ...
                robot.get_dim_configuration_space();
        end        
       
        function x = fkm(obj,q,ith)
        % x = fkm(q) receives the configuration vector q of the whole
        % kinematic chain and returns the pose of the last frame.
        % x = fkm(q, ith) calculates the forward kinematics up to the ith
        % kinematic chain.
            
            if nargin > 2
                n = ith;
            else
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
                dim = obj.chain{i}.get_dim_configuration_space();
                qi = q(j : j + dim - 1);
                j = j + dim;
                x = x*obj.chain{i}.fkm(qi);
            end
        end
        
        function J = pose_jacobian(obj,q,ith)
            
            if nargin > 2
                % find the jacobian up to the ith intermediate kinematic
                % chain
                n = ith;                
            else
                % find the jacobian of the whole chain
                n = length(obj.chain);
            end
            
            x_0_to_n = obj.fkm(q);
            j = 1;
            
            for i = 0:n-1
                x_0_to_iplus1 = obj.fkm(q,i+1);
                x_iplus1_to_n = x_0_to_iplus1'*x_0_to_n;
                
                dim = obj.chain{i+1}.get_dim_configuration_space();
                q_iplus1 = q(j : j + dim - 1);
                j = j + dim;
                L{i+1} = hamiplus8(obj.fkm(q,i))*haminus8(x_iplus1_to_n)*...
                    obj.chain{i+1}.pose_jacobian(q_iplus1);
            end
            J = cell2mat(L);
        end
        
        function plot(obj,q)
            dim_conf_space = obj.chain{1}.get_dim_configuration_space();
            plot(obj.chain{1},q(1:dim_conf_space));            
            
            j = dim_conf_space + 1;
            
            % Iterate over the chain
            for i = 2:length(obj.chain)
                % Replace n_links by dimension_configuration_space
                dim = obj.chain{i}.get_dim_configuration_space();
                qi = q(j : j + dim - 1);
                j = j + dim;        
                
                if isa(obj.chain{1}, 'DQ_MobileBase')
                    current_base_frame = obj.fkm(q,i-1);
                else
                    current_base_frame = obj.chain{1}.base_frame*obj.fkm(q,i-1);
                end
                
                % current_base_frame = obj.chain{1}.base_frame*obj.fkm(q,i-1);
                obj.chain{i}.set_base_frame(current_base_frame); 
                
                if i < length(obj.chain)
                plot(obj.chain{i},qi,'cylinder',[0,i*0.2,0], 'nobase',...
                    'nojoints', 'nowrist','noname');         
                else
                    plot(obj.chain{i},qi,'cylinder',[0,i*0.2,0], 'nobase',...
                    'nojoints','noname'); 
                end
                    
            end            
        end
        
        % Get configuration vector of the i-th element in the kinematic
        % chain
        % function ret = get_configuration(obj,q,i)
            
            
            
      %  end
    end    
end