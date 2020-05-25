classdef YouBotVrepRobot < DQ_VrepRobot
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        joint_names;
        base_frame_name;
    end
    
    properties (Constant)
        adjust = ((cos(pi/2) + DQ.i*sin(pi/2)) * (cos(pi/4) + DQ.j*sin(pi/4)))*(1+0.5*DQ.E*-0.1*DQ.k);
    end
    
    methods
        function obj = YouBotVrepRobot(robot_name,vrep_interface)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            %This might be useful to every subclass
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'youBot')
                error('Expected youBot')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            %Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:5
                current_joint_name = {robot_label,'ArmJoint',int2str(i-1),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = robot_name;
        end
        
        function send_q_to_vrep(obj,q)
            x = q(1);
            y = q(2);
            phi = q(3);
            
            r = cos(phi/2.0)+DQ.k*sin(phi/2.0);
            p = x * DQ.i + y * DQ.j;
            pose = (1 + DQ.E*0.5*p)*r;
            
            obj.vrep_interface.set_joint_positions(obj.joint_names,q(4:8));
            obj.vrep_interface.set_object_pose(obj.base_frame_name, pose * obj.adjust');
        end
        
        function q = get_q_from_vrep(obj)
            base_x = obj.vrep_interface.get_object_pose(obj.base_frame_name) * obj.adjust;
            base_t = vec3(translation(base_x));
            base_phi = rotation_angle(rotation(base_x));            
            base_arm_q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            
            q = [base_t(1); base_t(2); base_phi; base_arm_q];
        end
        
        function kin = kinematics(obj)
            include_namespace_dq
            % The DH parameters and other geometric parameters are based on
            % Kuka's documentation:
            % http://www.youbot-store.com/wiki/index.php/YouBot_Detailed_Specifications
            % https://www.generationrobots.com/img/Kuka-YouBot-Technical-Specs.pdf
            pi2 = pi/2;
            arm_DH_theta = [    0,    pi2,       0,      pi2,        0];
            arm_DH_d =   [  0.147,      0,       0,        0,    0.218];
            arm_DH_a =   [  0.033,  0.155,   0.135,        0,        0];
            arm_DH_alpha =   [pi2,      0,       0,      pi2,        0];
            arm_DH_matrix = [arm_DH_theta;
                arm_DH_d;
                arm_DH_a;
                arm_DH_alpha];
            
            arm =  DQ_SerialManipulator(arm_DH_matrix,'standard');
            base = DQ_HolonomicBase();
            
            x_bm = 1 + E_*0.5*(0.165*i_ + 0.11*k_);
            
            base.set_frame_displacement(x_bm);
            
            kin = DQ_WholeBody(base);
            kin.add(arm);
            
            effector = 1 + E_*0.5*0.3*k_;
            kin.set_effector(effector);
        end
        
    end
end

