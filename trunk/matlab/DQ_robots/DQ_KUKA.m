function ret = DQ_KUKA
    %Standard D-H of KUKA-LWR
    kuka_DH_theta=[0, 0, 0, 0, 0, 0, 0];
    kuka_DH_d = [0.310, 0, 0.4, 0, 0.39, 0, 0];
    kuka_DH_a = [0, 0, 0, 0, 0, 0, 0];
    kuka_DH_alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];
    kuka_DH_matrix = [kuka_DH_theta;
                      kuka_DH_d;
                      kuka_DH_a;
                      kuka_DH_alpha];

    ret = DQ_kinematics(kuka_DH_matrix,'standard');
end
