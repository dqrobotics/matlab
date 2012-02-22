clear all;
close all;
clc;


fprintf('\nLets start with the declaration of some arbitrary dual quaternions\n');

fprintf('\nCOMMAND: dq1 = DQ\n')
dq1 = DQ

fprintf('\nCOMMAND: dq2 = 2*DQ(1)\n')
dq2 = 2*DQ(1)

fprintf('\nCOMMAND: dq3 = DQ([1,2,3,4])\n')
dq3 = DQ([1,2,3,4])

fprintf('\nCOMMAND: dq4 = DQ([1,2,3,4,5,6,7,8])\n')
dq4 = DQ([1,2,3,4,5,6,7,8])

fprintf('\nRetrieving the real part of dq4. COMMAND: dq4.Re\n')
dq4.Re

fprintf('\nRetrieving the imaginary part of dq4. COMMAND: dq4.Im\n')
dq4.Im

fprintf('\nRetrieving the primary part of dq4. COMMAND: dq4.P\n')
dq4.P

fprintf('\nRetrieving the dual part of dq4. COMMAND: dq4.D\n')
dq4.D

fprintf('\nRetrieving the imaginary part of the dual part of dq4. COMMAND: dq4.Im.D\n')
dq4.Im.D

fprintf('\nRetrieving the j coefficient of the imaginary part of the dual part of dq4. COMMAND: dq4.Im.D(3)\n')
dq4.Im.D(3)

fprintf('\nThe dual unit is available too. COMMAND: DQ.E\n')
DQ.E

fprintf('\nRemember that the dual unit is nilpotent. COMMAND: DQ.E*DQ.E\n')
DQ.E*DQ.E

fprintf('\nThe imaginary units are also available. COMMAND: DQ.i, DQ.j, DQ.k\n');
DQ.i, DQ.j, DQ.k

fprintf('\nNow lets perform some operations\n');

fprintf('\nCOMMAND: (dq3+dq4)*dq2\n')
(dq3+dq4)*dq2

fprintf('\nThe same can be obtained in vector space using the Hamilton operators:\n');

fprintf('\nCOMMAND: hamiplus8(dq3+dq4)*vec8(dq2)\n');
hamiplus8(dq3+dq4)*vec8(dq2)

fprintf('\nCOMMAND: haminus8(dq2)*vec8(dq3+dq4)\n');
haminus8(dq2)*vec8(dq3+dq4)

fprintf('\nCOMMAND: dq3+dq4*dq2\n')
dq3+dq4*dq2

fprintf('\nCOMMAND: dq4*inv(dq4)\n')
dq4*inv(dq4)

fprintf('\nLets play with unit quaternions\n')
fprintf('\nCOMMAND: r1 = DQ([cos(pi/4),sin(pi/4),0,0])\n')
r1 = DQ([cos(pi/8),sin(pi/8),0,0])

fprintf('\nCOMMAND: DQ([cos(-pi/7),0,0,sin(-pi/7)])\n')
r2 = DQ([cos(-pi/7),0,0,sin(-pi/7)])

fprintf('\nCOMMAND: r3 = r1*r2\n')
r3 = r1*r2

fprintf('\nCOMMAND: norm(r3)\n')
norm(r3)

fprintf('\nLets play with unit dual quaternions\n')
fprintf('\nDeclaring a translation quaternion\n')
fprintf('\nCOMMAND: p = DQ([0,1,2,3])\n')
p = DQ([0,1,2,3])

fprintf('\nThe builtin dual unit (DQ.E) is quite handy:\n')
fprintf('\nCOMMAND: x = r1+DQ.E*p*r1*0.5\n')
x = r1+DQ.E*p*r1*0.5

fprintf('\nCOMMAND: norm(x)\n')
norm(x)

fprintf('\nCOMMAND: translation(x)\n')
translation(x)


fprintf('\nCOMMAND: translation(x)==p\n')
translation(x)==p

fprintf('\nCOMMAND: norm(x*x*x)\n')
norm(x*x*x)

fprintf('\nNow its time to go visual!\n')
fprintf('\nCOMMAND: plot(r1) hold on;\n')
figure; plot(r1); hold on;
xlabel('x'); ylabel('y'); zlabel('z');

fprintf('\nCOMMAND: plot(x)\n')
plot(x);



