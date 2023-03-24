function C = coriolis_matrix(q, q_dot, L1, L2)
% CORIOLIS_MATRIX computes the Coriolis matrix of a 2DOF RR manipulator

% Constants
m1 = 1;     % Mass of first link
m2 = 1;     % Mass of second link

% Elements of the Coriolis matrix
c11 = -m2*L1*L2*sin(q(2))*(2*q_dot(1) + q_dot(2));
c12 = -m2*L1*L2*sin(q(2))*q_dot(2);
c21 = m2*L1*L2*sin(q(2))*q_dot(1);
c22 = 0;

% Coriolis matrix
C = [c11 c12;
     c21 c22];
end
