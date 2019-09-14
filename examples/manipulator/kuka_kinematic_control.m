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
    x = kuka.fkm(q);
    e = epsilon+1;
    while norm(e) > epsilon
        J = kuka.pose_jacobian(q);
        x = kuka.fkm(q);
        e = vec8(xd-x);
        q = q+pinv(J)*gain*e;
        plot(kuka, q');    
        drawnow;
    end

    fprintf('\nNow let us control only the translation part\n');
    %The end-effector will touch the base
    pd = 0*DQ.i + 0*DQ.j + 0*DQ.k;

    e = epsilon+1;
    while norm(e) > epsilon
        J = kuka.pose_jacobian(q);
        x = kuka.fkm(q);
        jacobp = kuka.translation_jacobian(J,x);
        pm = translation(x);
        e = vec4(pd-pm);    
        q = q+pinv(jacobp)*gain*e;
        plot(kuka, q'); 
        drawnow;
    end

    fprintf('\nNow let us control only the orientation\n')

    %The end-effector will be aligned with the world frame
    rd = DQ(1);

    e = epsilon+1;
    while norm(e) > epsilon
        J = kuka.pose_jacobian(q);
        x = kuka.fkm(q);
        jacobr = kuka.rotation_jacobian(J);
        rm = x.P;
        e = vec4(rd-rm);    
        q = q+pinv(jacobr)*gain*e;
        plot(kuka, q');  
        drawnow;
    end


    fprintf('\nNow let us place the end-effector at a distance of 0.2 m from the base (we are going to perform distance control)\n')

    %Technically speaking, we're controlling the square of the distance,
    %otherwise the distance Jacobian can have singularities. See discussion on page 76 of 
    %ADORNO, B. V., Two-arm manipulation: from manipulators to enhanced human-robot
    % collaboration, Universit? Montpellier 2, Montpellier, France, 2011.
    dd=0.2^2;
    e = epsilon+1;
    while norm(e) > epsilon
        J = kuka.pose_jacobian(q);
        x = kuka.fkm(q);
        jacobd = kuka.distance_jacobian(J,x);
        dm = norm(vec4(translation(x)))^2;
        e = dd-dm;    
        q = q+pinv(jacobd)*gain*e;
        plot(kuka, q');
        drawnow;
    end
end




