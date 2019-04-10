function wam_kinematic_control()

    %Create a new DQ_kinematics object with the WAM arm standard Denavit-Hartenberg parameters                              
    wam = BarrettWamArmRobot.kinematics();

    %Initial configuration
    q =[-2.65    -0.604    -0.46    2.68        1.35         1.61         -3.03]';
    %Final configuration
    qd = [-2.0    -0.304    -0.3    2.3        1.0         1.0         -2.03]';

    epsilon = 0.001; %The error must be bellow this value in order to stop the robot
    gain = 0.1; %Gain of the controllers

    xd = wam.fkm(qd); %Desired end-effector's pose

    figure;
    axis equal;
    plot(wam, q);

    grid off;
    view(-0,0)
    hold on;

    plot(wam, q);

    disp('Performing standard kinematic control using dual quaternion coordinates');
    xm = wam.fkm(q);
    error = epsilon+1;
    while norm(error) > epsilon
        jacob = wam.pose_jacobian(q);
        xm = wam.fkm(q);
        error = vec8(xd-xm);
        q = q+pinv(jacob)*gain*error;
        plot(wam, q');    
        drawnow;
    end
end
