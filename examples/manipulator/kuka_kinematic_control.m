function kuka_kinematic_control()

    %Create a new DQ_kinematics object with KUKA LWR parameters
    kuka = KukaLwr4Robot.kinematics();
    kuka.name = 'KUKA';

    %Initial configuration
    qstart =[0    0.3770    0.1257   -0.5655         0         0         0]';
    %Final configuration
    qd = [1.7593    0.8796    0.1257   -1.4451   -1.0053    0.0628         0]';
    q = qstart;

    epsilon = 0.001; %The error must be bellow this value in order to stop the robot
    gain = 0.1; %Gain of the controllers

    xd = kuka.fkm(qd); %Desired end-effector's pose

    figure;
    axis equal;
    plot(kuka, q);

    grid off;
    view(-0,0)
    hold on;

    plot(kuka, q);


    fprintf('Performing standard kinematic control using dual quaternion coordinates');
    xm = kuka.fkm(q);
    error = epsilon+1;
    while norm(error) > epsilon
        jacob = kuka.pose_jacobian(q);
        xm = kuka.fkm(q);
        error = vec8(xd-xm);
        q = q+pinv(jacob)*gain*error;
        plot(kuka, q');    
        drawnow;
    end

    fprintf('\nNow let us control only the translation part\n');
    %The end-effector will touch the base
    pd = 0*DQ.i + 0*DQ.j + 0*DQ.k;

    error = epsilon+1;
    while norm(error) > epsilon
        jacob = kuka.pose_jacobian(q);
        xm = kuka.fkm(q);
        jacobp = kuka.translation_jacobian(jacob,xm);
        pm = translation(xm);
        error = vec4(pd-pm);    
        q = q+pinv(jacobp)*gain*error;
        plot(kuka, q'); 
        drawnow;
    end

    fprintf('\nNow let us control only the orientation\n')

    %The end-effector will be aligned with the world frame
    rd = DQ(1);

    error = epsilon+1;
    while norm(error) > epsilon
        jacob = kuka.pose_jacobian(q);
        xm = kuka.fkm(q);
        jacobr = kuka.rotation_jacobian(jacob);
        rm = xm.P;
        error = vec4(rd-rm);    
        q = q+pinv(jacobr)*gain*error;
        plot(kuka, q');  
        drawnow;
    end


    fprintf('\nNow let us place the end-effector at a distance of 0.2 m from the base (we are going to perform distance control)\n')

    %Technically speaking, we're controlling the square of the distance,
    %otherwise the distance Jacobian can have singularities. See discussion on page 76 of 
    %ADORNO, B. V., Two-arm manipulation: from manipulators to enhanced human-robot
    % collaboration, Universit? Montpellier 2, Montpellier, France, 2011.
    dd=0.2^2;
    error = epsilon+1;
    while norm(error) > epsilon
        jacob = kuka.pose_jacobian(q);
        xm = kuka.fkm(q);
        jacobd = kuka.distance_jacobian(jacob,xm);
        dm = norm(vec4(translation(xm)))^2;
        error = dd-dm;    
        q = q+pinv(jacobd)*gain*error;
        plot(kuka, q');
        drawnow;
    end
end




