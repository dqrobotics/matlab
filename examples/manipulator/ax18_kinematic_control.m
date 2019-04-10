function ax18_kinematic_control()

    % Create a new DQ_kinematics object with the AX18 arm standard
    % Denavit-Hartenberg parameters
    ax18 = Ax18ManipulatorRobot.kinematics();

    % Initial configuration
    theta=[0 0 0 0 0]';

    % Configuring the end-effector:
    ax18.set_effector(cos(pi/4)+DQ.i*sin(pi/4));

    % Generate a desired pose base on final joint angles. In real applications,
    % usually we don't have the desired joint configurations.
    xd = ax18.fkm([pi/4 -pi/4 pi/2 0 pi/4]);

    x = ax18.fkm(theta);

    % Let us plot the desired pose
    plot(xd, 'scale', 0.1);

    % This is a very simple way for calculating the error, but it has some
    % problems. For instance, it does not respect the topology of the unit dual
    % quaternion space.
    error = xd - x;
    while norm(vec4(error)) > 0.01
        x = ax18.fkm(theta);
        J = ax18.pose_jacobian(theta);
        error = xd - x;
        theta = theta + pinv(J)*0.05*vec8(error);
        plot(ax18,theta);
        drawnow;
    end
end
