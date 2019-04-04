classdef DQ_WholeBody < handle
    properties %(Access = protected)
        chain;
    end
    
    methods
        function obj = DQ_WholeBody(robot)
            % TODO: test if robot is an instance of DQ_Kinematics
            obj.chain{1} = robot;
        end
        
        function add(obj, robot)
            % add(robot) adds a robot to the end of the serial kinematic chain
            % TODO: test if robot is an instance of DQ_Kinematics
            len = length(obj.chain);
            obj.chain{len + 1} = robot;
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
                dim = obj.chain{i}.n_links;
                qi = q(j : j + dim - 1);
                j = j + dim;
                x = x*obj.chain{i}.fkm(qi);
            end
        end
        
        function plot(obj,q)
            
            current_base_frame = obj.chain{1}.base_frame;
            n_links = obj.chain{1}.n_links;
            plot(obj.chain{1},q(1:n_links),'cylinder',[0, 0, 0]);            
            
            j = n_links + 1;
            
            % Iterate over the chain
            for i = 2:length(obj.chain)
                % Replace n_links by dimension_configuration_space
                dim = obj.chain{i}.n_links;
                qi = q(j : j + dim - 1);
                j = j + dim;               
                
                current_base_frame = obj.chain{1}.base_frame*obj.fkm(q,i-1);
                obj.chain{i}.set_base_frame(current_base_frame);                    
                plot(obj.chain{i},qi,'cylinder',[0,i*0.2,0], 'nobase');         
            end            
        end
        
        % Get configuration vector of the i-th element in the kinematic
        % chain
        % function ret = get_configuration(obj,q,i)
            
            
            
      %  end
    end    
end