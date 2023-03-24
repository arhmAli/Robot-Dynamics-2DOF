clc; clear; close all;

% Manipulator Parameters
L1 = 3; % Length of first link
L2 = 3; % Length of second link

% Trajectory Parameters
tf=2*pi;
traj = 0:0.01:tf; % Time vector
%[x,y,vx,vy,ax,ay]=Circle_Traj(traj(1),tf,tf,0);
 x = cos(traj); % X coordinate of the trajectory
 y = sin(traj); % Y coordinate of the trajectory
 vx = -sin(traj); % X velocity of the trajectory
 vy = cos(traj); % Y velocity of the trajectory
 ax = -cos(traj); % X acceleration of the trajectory
 ay = -sin(traj); % Y acceleration of the trajectory

% Simulation Parameters
dt = traj(2)-traj(1); % Time step
q0 = [-pi/6; pi/6]; % Initial joint angles
q_dot0 = [0; 0]; % Initial joint velocities
q_ddot0 = [0; 0]; % Initial joint accelerations

% Simulation
q = q0;
q_dot = q_dot0;
q_ddot = q_ddot0;


figure;
hold on;
axis equal;
xlim([-7, 7]);
ylim([-7, 7]);
xlabel('X');
ylabel('Y');
title('2DOF RR Manipulator Trajectory Following using Inverse Kinematics');
for i = 1:length(traj)-1
    
    % Desired end-effector position and Jacobian matrix
    [H1,H2,J] = hGen(q, L1, L2);
    angles=InverseKinematics(x(i),y(i));
    p_d = angles
    p_dot_d = J*[vx(i); vy(i)];
   %//some=([ax(i);ay(i)]-(p_dot_d)'.*[H1;H2].*(p_dot_d));
    p_ddot_d =J*inv(p_d);
    % Joint torques 
    M = mass_matrix(q, L1, L2);
    V = coriolis_matrix(q, q_dot, L1, L2);
    G = gravity_vector(q, L1, L2);
    tau = M*p_ddot_d + V*p_dot_d + G;
    % Update joint accelerations, velocities and angles using inverse kinematics
    % Current end-effector position, velocity and acceleration
	qCurrIni=dhparams(L1,L2,angles(1,2),angles(2,2));
    qCurr=[qCurrIni(1,1) qCurrIni(1,2); qCurrIni(2,1) qCurrIni(2,2)];
    q_dotCurr_1=diff(qCurr);
    q_dotCurr=[q_dotCurr_1;x(i),y(i)];
	q_ddotCurr = M  *inv(qCurr)* ( tau - coriolis_matrix(qCurr,q_dotCurr,L1,L2) - gravity_vector(qCurr,L1,L2));
    % Current joint angle error
    e = p_d - qCurr;
    e_dot = p_dot_d - J*q_dotCurr;
    e_ddot = p_ddot_d - J*q_ddotCurr;
    
    %Integration
    int_1=diff(q_dotCurr);
    int_2=diff(q_ddotCurr);
    int=[int_1;int_2];
    % Plot the manipulator and trajectory
    x_base = 0;
    y_base = 0;
    x_ee = L1*cos(qCurr(1,2)) + L2*cos(qCurr(2,2)+qCurr(2,2));
    y_ee = L1*sin(qCurr(1,2)) + L2*sin(qCurr(2,2)+qCurr(2,2));
   plot([x_base, L1*cos(qCurr(1,2)), x_ee], [y_base, L1*sin(qCurr(2,2)), y_ee], 'LineWidth', 1);
    plot(x_ee, y_ee, 'bo', 'MarkerSize', 2, 'MarkerFaceColor', 'b');
    %legend({'Manipulator', 'Desired Trajectory', 'Desired End Effector', 'Actual End Effector'}, 'Location', 'southwest');
    drawnow ;
    pause(0.01);
end


fprintf("\nThe Desired joint position is  \npDesired=[%d\t%d\t%d]\n",p_d);
fprintf("\nThe current joint position is  \npActual=[%d\t%d\t%d]\n",qCurr);

fprintf("\nThe Desired joint velocity is  \nvDesired=[%d\t%d\t%d]\n",p_dot_d);
fprintf("\nThe Actual joint velocity is  \nvDesired=[%d\t%d\t%d]\n",q_dotCurr);
fprintf("\nThe error matrix is given for velocity as \nvError=[%d\t%d\t%d]\n",e_dot);


fprintf("\nThe  Desired joint acceleration is as \naDesired=[%d\t%d\t%d]\n]",p_ddot_d);
fprintf("\nThe Actual joint accleration is  \nvDesired=[%d\t%d\t%d]\n",q_ddotCurr);
fprintf("\nThe error matrix is given for accleration is as \naError=[%d\t%d\t%d]\n]",e_ddot);

fprintf("\nThe tau matrix is given as \nTau=[%d\t%d\t%d]\n]",tau);

