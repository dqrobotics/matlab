% comau = DQ_COMAU returns a DQ_kinematics object using the modified
% Denavit-Hartenberg parameters of the COMAU SmartSiX robot
function comau = DQ_COMAU
   %% Definitions for DQ_kinematics
    comau_DH_theta=  [0, -pi/2, pi/2, 0, 0, 0, pi];
    comau_DH_d =     [-0.45, 0, 0, -0.64707, 0, -0.095, 0];
    comau_DH_a =     [0, 0.150, 0.590, 0.13, 0, 0, 0];
    comau_DH_alpha = [pi, pi/2, pi, -pi/2, -pi/2, pi/2, pi];
    comau_dummy =    [0,0,0,0,0,0,1];

    comau_DH_matrix = [comau_DH_theta;
        comau_DH_d;
        comau_DH_a;
        comau_DH_alpha;
        comau_dummy];

    comau = DQ_kinematics(comau_DH_matrix, 'modified');

end