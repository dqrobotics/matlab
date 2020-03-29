function wam_kinematic_control()
load('wam_kinematic_control.mat','store_pose_jacobian','store_fkm','xd');

%Create a new DQ_kinematics object with the WAM arm standard Denavit-Hartenberg parameters
wam = BarrettWamArmRobot.kinematics();

%Initial configuration
q =[-2.65    -0.604    -0.46    2.68        1.35         1.61         -3.03]';
%Final configuration
qd = [-2.0    -0.304    -0.3    2.3        1.0         1.0         -2.03]';

epsilon = 0.001; %The error must be bellow this value in order to stop the robot
gain = 0.1; %Gain of the controllers

if xd~=wam.fkm(qd)
    error('Error in xd')
end
xd = wam.fkm(qd); %Desired end-effector's pose

figure;
axis equal;
plot(wam, q);

grid off;
view(-0,0)
hold on;

plot(wam, q);

disp('Performing standard kinematic control using dual quaternion coordinates');
err = epsilon+1;
%store_fkm = {};
%store_pose_jacobian = {};
i=1;
while norm(err) > epsilon
    jacob = wam.pose_jacobian(q);
    if jacob~=store_pose_jacobian{i}
        jacob
        store_pose_jacobian{i}
        error(['Error in pose_jacobian at i=' int2str(i)])
    end
    xm = wam.fkm(q);
    if xm~=store_fkm{i}
        error(['Error in fkm at i=' int2str(i)])
    end
    %jacob = wam.pose_jacobian(q);
    %store_pose_jacobian{i} = jacob;
    %xm = wam.fkm(q);
    %store_fkm{i} = xm;
    err = vec8(xd-xm);
    q = q+pinv(jacob)*gain*err;
    plot(wam, q');
    drawnow;
    i=i+1;
end
%save('wam_kinematic_control.mat')
end
