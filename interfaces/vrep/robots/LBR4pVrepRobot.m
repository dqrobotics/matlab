classdef LBR4pVrepRobot < DQ_VrepRobot
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        joint_names;
        base_frame_name;
    end
    
    methods
        function obj = LBR4pVrepRobot(robot_name,vrep_interface)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            %This might be useful to every subclass
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'LBR4p')
                error('Expected LBR4p')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            %Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:7
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end
        
        
        function send_q_to_vrep(obj,q)
            obj.vrep_interface.set_joint_positions(obj.joint_names,q)
        end
        
        function q = get_q_from_vrep(obj)
            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
        end
        
        function kin = kinematics(obj)
            %Standard D-H of LBR4p
            LBR4p_DH_theta=[0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_d = [0.200, 0, 0.4, 0, 0.39, 0, 0];
            LBR4p_DH_a = [0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_alpha = [pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0];
            LBR4p_DH_matrix = [LBR4p_DH_theta;
                LBR4p_DH_d;
                LBR4p_DH_a;
                LBR4p_DH_alpha];
            
            kin = DQ_SerialManipulator(LBR4p_DH_matrix,'standard');
            % We set the transformation from the world frame to the robot
            % base frame. Therefore, the end-effector pose is given by
            % pose_effector = transformation_from_world_to_base*fkm(q);
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_effector(1+0.5*DQ.E*DQ.k*0.07);
        end
        
    end
end

