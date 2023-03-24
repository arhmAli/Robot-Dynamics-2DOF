function G = gravity_vector(q, L1, L2)
% GRAVITY_VECTOR computes the gravity vector for a 2DOF RR manipulator
% using the physical parameters of the links

% Joint angles
q1 = q(1);
q2 = q(2);

% Physical parameters of the links
m1 = 1;
m2 = 0.8;
g = 9.81;
r1 = L1/2;
r2 = L2/2;

% Gravity vector
G1 = (m1*r1 + m2*L1)*g*cos(q1) + m2*r2*g*cos(q1+q2);
G2 = m2*r2*g*cos(q1+q2);
G = [G1;
     G2];
end