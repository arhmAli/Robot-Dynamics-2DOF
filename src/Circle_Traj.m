function [x,y,vx,vy,ax,ay] = Circle_Traj(t,tf,th_tf,th_t0);

a_0=0;
a_1=0;
a_2=3*(th_tf-th_t0)/(tf^2);
a_3=2*(th_tf-th_t0)/(tf^3);
theta_Traj=a_0+(a_1)*(t)+(a_2*t^2)+(a_3)*(t^3);
r=1;
a=2;
b=2;
x =r*sin(theta_Traj)+a ;
y = r*cos(theta_Traj)+b;
vx=2*cos(theta_Traj);
vy=r*sin(theta_Traj);
ax=r*cos(theta_Traj);
ay=-2*sin(theta_Traj);
end