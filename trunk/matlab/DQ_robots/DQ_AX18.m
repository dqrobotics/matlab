% AX = DQ_AX18 returns a DQ_kinematics object using the standard
% Denavit-Hartenberg parameters of the AX18 robot
function ax = DQ_AX18
    %Standard D-H of AX18 arm
    ax18_DH_theta   = [0      0      -pi/2   0      -pi/2   0];  % theta
    ax18_DH_d       = [167    0       0      81.5    41     0];  % d
    ax18_DH_a       = [0      159     0      22.25   0      0];  % a
    ax18_DH_alpha   = [-pi/2  0       -pi/2  0      -pi/2   0];  % alpha
    
    % definition of virtual joints. (Joints that are not actuated and are 
    % used only to simplify the parameters definition. A more appropriate
    % parameters set, which includes only the five actual joints should be 
    % derived in the future.)
    ax18_DH_virtual = [0,     0,      0,     1,      0,     0]; 

    ax18_DH_matrix = [  ax18_DH_theta; 
                        ax18_DH_d;
                        ax18_DH_a; 
                        ax18_DH_alpha; 
                        ax18_DH_virtual]; % D&H parameters matrix for the arm model

    ax = DQ_kinematics(ax18_DH_matrix,'standard'); % Defines robot model using dual quaternions

end