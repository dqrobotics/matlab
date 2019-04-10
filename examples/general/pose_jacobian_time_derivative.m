% Simple example to compare the analytical solution for the time derivative
% of the Jacobian matrix and the numerical one.

function pose_jacobian_time_derivative()
    % Create a new DQ_kinematics object with KUKA LWR parameters
    kuka = DQ_KUKA;

    % Integration step for the numerical calculations
    T = 1e-3;
    % Final time
    T_end = 2*pi;

    theta = zeros(7,1);
    theta_plot = zeros(1,ceil(T_end/T)); 
    max_theta_plot = length(theta_plot);

    j = 1;
    for t = 0:T:T_end
        % For simplicity, all joint trajectories are the same. All joints
        % rotate at a frequency of T rad/s.
        theta = sin(T*t)*ones(7,1);
        % This is the analytical time derivative of the joint trajectories. 
        theta_dot = T*cos(t)*ones(7,1);
        % Calculation of the analytical Jacobian time derivative.    
        jacob_dot = kuka.pose_jacobian_derivative(theta,theta_dot);
        % First-order numerical approximation of the Jacobian time derivative.
        jacob_diff = (kuka.raw_pose_jacobian(theta + theta_dot*T) - kuka.raw_pose_jacobian(theta))/T;
        % We store the Frobenius norm of the difference between the two
        % matrices. The smaller the difference, the better.
        theta_plot(j) = norm(jacob_dot - jacob_diff,'fro');
        j = j+1;

        fprintf('%d/%d\n',j,max_theta_plot);

    end
    plot(theta_plot);
end
