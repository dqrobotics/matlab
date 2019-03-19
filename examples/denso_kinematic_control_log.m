
clear all;
close all;
clear classes;
clc;

%Create a new DQ_kinematics object with the Denso standard Denavit-Hartenberg parameters           
denso_kine = DQ_DENSO;

theta = [0,0,0,0,0,0]';

% Move Arm

position = [0.4,0.0,0.02];

thetad = [0,0,0,0,0,0]';

phi = pi/2;

n_y =1;
n_x =0;
n_z =0;

n_vec = [n_x,n_y,n_z];
n = n_vec/norm(n_vec);

n = DQ([0, n_vec(1), n_vec(2), n_vec(3)]);

theta1 = atan2(position(2), position(1));
rz = cos(theta1/2) + sin(theta1/2)*DQ([0, 0, 0, 1]);

ry = cos(pi/4) + sin(pi/4)*DQ([0, 0, 1, 0]);

r = cos(phi/2) + sin(phi/2)*n;

r = ry * r;

p = DQ([0, position(1), position(2), position(3)]);

xd = r + 1/2 * DQ.E * p * r;

epsilon = 0.001;
K = 0.5;
error = epsilon+1;
lambda = 0.5;

% Kinematic control with maping log

r_haminus4 = haminus4(r);

p_hamiplus4 =hamiplus4(p);


y = log(r);

axis equal;
axis([-0.8,1.2,-0.8,0.8,-0.2,1.5]);
% zlim(axes1,[-0.69 0.69]);


pause(1);


while norm(error) > epsilon  
    dqy = norm(y)
    a = sin(dqy.q(1))/dqy.q(1);
    b = (cos(dqy.q(1))/dqy.q(1)^2)-(sin(dqy.q(1))/dqy.q(1)^3);

    dvec4r_dvec3y = [-a*y.q(2) -a*y.q(3) -a*y.q(4);
                b*y.q(2)^2+a b*y.q(3)*y.q(4) b*y.q(2)*y.q(4);
                b*y.q(2)*y.q(3) b*y.q(3)^2+a b*y.q(3)*y.q(4);
                b*y.q(2)*y.q(4) b*y.q(3)*y.q(4) b*y.q(4)^2+a];

    Q = dvec4r_dvec3y;
    Qp = [zeros(1,3);eye(3)];
    
    Q8 = [Q zeros(4,3);(1/2)*p_hamiplus4*Q r_haminus4*Qp];
    
    jacob = denso_kine.jacobian(theta);
    xm = denso_kine.fkm(theta);
    ym = log(xm);
    yd = log(xd);
    error = (yd-ym);
    error = [error.q(2) error.q(3) error.q(4) error.q(6) error.q(7) error.q(8)]';
    
    jacob_pinv = (transpose(jacob)/(jacob*transpose(jacob) + (lambda^2)*eye(8)));
        
     
    theta = theta + K*jacob_pinv*Q8*error;
    
    dt = 0.01;
       
    
    norm(error)
    plot(denso_kine, theta');
    plot(xm,'scale',0.2);
    hold on
    axis equal;
axis([-0.8,1.2,-0.8,0.8,-0.2,1.5]);
    view(-0.5 ,0);
    %pause(0.2)

    drawnow;

end
