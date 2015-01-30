function wam = DQ_WAM
%Standard D-H of WAM arm
    wam_DH_theta=[0, 0, 0, 0, 0, 0, 0];
    wam_DH_d = [0, 0, 0.55, 0, 0.3, 0, 0.0609];
    wam_DH_a = [0, 0, 0.045, -0.045, 0, 0, 0];
    wam_DH_alpha =  pi*[-0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0];
    wam_DH_matrix = [wam_DH_theta;
                      wam_DH_d;
                      wam_DH_a;
                      wam_DH_alpha];

    wam = DQ_kinematics(wam_DH_matrix,'standard');
end