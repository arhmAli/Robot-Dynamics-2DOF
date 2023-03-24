function M = mass_matrix(q, L1, L2)
% MASS_MATRIX computes the mass matrix for a 2DOF RR manipulator using the
% physical parameters of the links

% Joint angles
q1 = q(1);
q2 = q(2);

% Physical parameters of the links
m1 = 1;
m2 = 0.8;
I1 = 0.2;
I2 = 0.1;
r1 = L1/2;
r2 = L2/2;

% Mass matrix
M11 = m1*r1^2 + m2*(L1^2 + r2^2 + 2*L1*r2*cos(q2)) + I1 + I2;
M12 = m2*(r2^2 + L1*r2*cos(q2)) + I2;
M21 = M12;
M22 = m2*r2^2 + I2;
M = [M11, M12;
     M21, M22];
end