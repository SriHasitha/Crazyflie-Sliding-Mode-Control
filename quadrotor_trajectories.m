clear;clc;
%% Trajectory Generation
%declare variables
syms t 'real'
syms a0 a1 a2 a3 a4 a5 'real'
syms b0 b1 b2 b3 b4 b5 'real'
syms c0 c1 c2 c3 c4 c5 'real'
var = [a0,a1,a2,a3,a4,a5;b0,b1,b2,b3,b4,b5;c0,c1,c2,c3,c4,c5];
%trajectories
x = [a0,a1,a2,a3,a4,a5]*[1,t,t^2,t^3,t^4,t^5]';
y = [b0,b1,b2,b3,b4,b5]*[1,t,t^2,t^3,t^4,t^5]';
z = [c0,c1,c2,c3,c4,c5]*[1,t,t^2,t^3,t^4,t^5]';
q = [x,y,z];
x_dot = diff(x);
y_dot = diff(y);
z_dot = diff(z);
q_dot = [x_dot,y_dot,z_dot];
x_ddot = diff(x_dot);
y_ddot = diff(y_dot);
z_ddot = diff(z_dot);
q_ddot = [x_ddot,y_ddot,z_ddot];
%time
time = [0,5,20,35,50,65];
t0 = 0;
t1 = t0 + 5;
t2 = t1 + 15;
t3 = t2 + 15;
t4 = t3 + 15;
t5 = t4 + 15;
%waypoints
p0 = [0,0,0];
p1 = [0,0,1];
p2 = [1,0,1];
p3 = [1,1,1];
p4 = [0,1,1];
p5 = [0,0,1];
p = [p0;p1;p2;p3;p4;p5];
a=[];
Q = sym('Q',[5,3]);
Q_dot = sym('Q_dot',[5,3]);
Q_ddot = sym('Q_ddot',[5,3]);
%solving for coeffs
for j = 1:3
    for i = 1:5
        ex1 = subs(q(j),t,time(i))==p(i,j);
        ex2 = subs(q_dot(j),t,time(i))==0;
        ex3 = subs(q_ddot(j),t,time(i))==0;
        ex4 = subs(q(j),t,time(i+1))==p(i+1,j);
        ex5 = subs(q_dot(j),t,time(i+1))==0;
        ex6 = subs(q_ddot(j),t,time(i+1))==0;
        [coeff,b] = equationsToMatrix([ex1,ex2,ex3,ex4,ex5,ex6],var(j,:));
        a(i,:,j) = (double(linsolve(coeff,b)))';
        Q(i,j) = subs(q(j),var(j,:),a(i,:,j));
        if Q(i,j) == 0
            Q_dot(i,j) = 0;
        else
            Q_dot(i,j) = diff(Q(i,j));
        end
        if Q_dot(i,j) == 0
            Q_ddot(i,j) = 0;
        else
            Q_ddot(i,j) = diff(Q_dot(i,j));
        end
     end
end
%Symblic traj equations
%X
x1 = simplify(subs(x,[a0,a1,a2,a3,a4,a5],a(1,:,1)));
x1_dot = diff(x1);
x1_ddot = diff(x1_dot);
x2 = simplify(subs(x,[a0,a1,a2,a3,a4,a5],a(2,:,1)));
x2_dot = diff(x2);
x2_ddot = diff(x2_dot);
x3 = simplify(subs(x,[a0,a1,a2,a3,a4,a5],a(3,:,1)));
x3_dot = diff(x3);
x3_ddot = diff(x3_dot);
x4 = simplify(subs(x,[a0,a1,a2,a3,a4,a5],a(4,:,1)));
x4_dot = diff(x4);
x4_ddot = diff(x4_dot);
x5 = simplify(subs(x,[a0,a1,a2,a3,a4,a5],a(5,:,1)));
x5_dot = diff(x5);
x5_ddot = diff(x5_dot);
%y
y1 = simplify(subs(y,[b0,b1,b2,b3,b4,b5],a(1,:,2)));
y1_dot = diff(y1);
y1_ddot = diff(y1_dot);
y2 = simplify(subs(y,[b0,b1,b2,b3,b4,b5],a(2,:,2)));
y2_dot = diff(y2);
y2_ddot = diff(y2_dot);
y3 = simplify(subs(y,[b0,b1,b2,b3,b4,b5],a(3,:,2)));
y3_dot = diff(y3);
y3_ddot = diff(y3_dot);
y4 = simplify(subs(y,[b0,b1,b2,b3,b4,b5],a(4,:,2)));
y4_dot = diff(y4);
y4_ddot = diff(y4_dot);
y5 = simplify(subs(y,[b0,b1,b2,b3,b4,b5],a(5,:,2)));
y5_dot = diff(y5);
y5_ddot = diff(y5_dot);
%z
z1 = simplify(subs(z,[c0,c1,c2,c3,c4,c5],a(1,:,3)));
z1_dot = diff(z1);
z1_ddot = diff(z1_dot);
z2 = simplify(subs(z,[c0,c1,c2,c3,c4,c5],a(2,:,3)));
z2_dot = diff(z2);
z2_ddot = diff(z2_dot);
z3 = simplify(subs(z,[c0,c1,c2,c3,c4,c5],a(3,:,3)));
z3_dot = diff(z3);
z3_ddot = diff(z3_dot);
z4 = simplify(subs(z,[c0,c1,c2,c3,c4,c5],a(4,:,3)));
z4_dot = diff(z4);
z4_ddot = diff(z4_dot);
z5 = simplify(subs(z,[c0,c1,c2,c3,c4,c5],a(5,:,3)));
z5_dot = diff(z5);
z5_ddot = diff(z5_dot);

%trajectory plots
ts = t0:0.1:t5;
x_traj = [subs(Q(1,1),t,t0:0.1:t1),subs(Q(2,1),t,t1+0.1:0.1:t2),subs(Q(3,1),t,t2+0.1:0.1:t3),subs(Q(4,1),t,t3+0.1:0.1:t4),subs(Q(5,1),t,t4+0.1:0.1:t5)]; 
xdot_traj = [subs(Q_dot(1,1),t,t0:0.1:t1),subs(Q_dot(2,1),t,t1+0.1:0.1:t2),subs(Q_dot(3,1),t,t2+0.1:0.1:t3),subs(Q_dot(4,1),t,t3+0.1:0.1:t4),subs(Q_dot(5,1),t,t4+0.1:0.1:t5)]; 
xddot_traj = [subs(Q_ddot(1,1),t,t0:0.1:t1),subs(Q_ddot(2,1),t,t1+0.1:0.1:t2),subs(Q_ddot(3,1),t,t2+0.1:0.1:t3),subs(Q_ddot(4,1),t,t3+0.1:0.1:t4),subs(Q_ddot(5,1),t,t4+0.1:0.1:t5)]; 
y_traj = [subs(Q(1,2),t,t0:0.1:t1),subs(Q(2,2),t,t1+0.1:0.1:t2),subs(Q(3,2),t,t2+0.1:0.1:t3),subs(Q(4,2),t,t3+0.1:0.1:t4),subs(Q(5,2),t,t4+0.1:0.1:t5)]; 
ydot_traj = [subs(Q_dot(1,2),t,t0:0.1:t1),subs(Q_dot(2,2),t,t1+0.1:0.1:t2),subs(Q_dot(3,2),t,t2+0.1:0.1:t3),subs(Q_dot(4,2),t,t3+0.1:0.1:t4),subs(Q_dot(5,2),t,t4+0.1:0.1:t5)]; 
yddot_traj = [subs(Q_ddot(1,2),t,t0:0.1:t1),subs(Q_ddot(2,2),t,t1+0.1:0.1:t2),subs(Q_ddot(3,2),t,t2+0.1:0.1:t3),subs(Q_ddot(4,2),t,t3+0.1:0.1:t4),subs(Q_ddot(5,2),t,t4+0.1:0.1:t5)];
z_traj = [subs(Q(1,3),t,t0:0.1:t1),subs(Q(2,3),t,t1+0.1:0.1:t2),subs(Q(3,3),t,t2+0.1:0.1:t3),subs(Q(4,3),t,t3+0.1:0.1:t4),subs(Q(5,3),t,t4+0.1:0.1:t5)]; 
zdot_traj = [subs(Q_dot(1,3),t,t0:0.1:t1),subs(Q_dot(2,3),t,t1+0.1:0.1:t2),subs(Q_dot(3,3),t,t2+0.1:0.1:t3),subs(Q_dot(4,3),t,t3+0.1:0.1:t4),subs(Q_dot(5,3),t,t4+0.1:0.1:t5)]; 
zddot_traj = [subs(Q_ddot(1,3),t,t0:0.1:t1),subs(Q_ddot(2,3),t,t1+0.1:0.1:t2),subs(Q_ddot(3,3),t,t2+0.1:0.1:t3),subs(Q_ddot(4,3),t,t3+0.1:0.1:t4),subs(Q_ddot(5,3),t,t4+0.1:0.1:t5)];

figure('Name','Desired Trajectories for Translational Coordinate x')
subplot(3,1,1)
plot(ts,x_traj)
title('Desired Position Trajectory')
xlabel('time')
ylabel('x(t)')
xlim([0,65])
ylim([-0.5,1.5])
subplot(3,1,2)
plot(ts,xdot_traj)
title('Desired Velocity Trajectory')
xlabel('time')
ylabel('x\_dot(t)')
xlim([0,65])
ylim([-0.2,0.2])
subplot(3,1,3)
plot(ts,xddot_traj)
title('Desired Acceleration Trajectory')
xlabel('time')
ylabel('x\_ddot(t)')
xlim([0,65])
ylim([-0.04,0.04])

figure('Name','Desired Trajectories for Translational Coordinate y')
subplot(3,1,1)
plot(ts,y_traj)
title('Desired Position Trajectory')
xlabel('time')
ylabel('y(t)')
xlim([0,65])
ylim([-0.5,1.5])
subplot(3,1,2)
plot(ts,ydot_traj)
title('Desired Velocity Trajectory')
xlabel('time')
ylabel('y\_dot(t)')
xlim([0,65])
ylim([-0.2,0.2])
subplot(3,1,3)
plot(ts,yddot_traj)
title('Desired Acceleration Trajectory')
xlabel('time')
ylabel('y\_ddot(t)')
xlim([0,65])
ylim([-0.04,0.04])

figure('Name','Desired Trajectories for Translational Coordinate z')
subplot(3,1,1)
plot(ts,z_traj)
title('Desired Position Trajectory')
xlabel('time')
ylabel('z(t)')
xlim([0,65])
ylim([-0.5,1.5])
subplot(3,1,2)
plot(ts,zdot_traj)
title('Desired Velocity Trajectory')
xlabel('time')
ylabel('z\_dot(t)')
xlim([0,65])
ylim([-0.6,0.6])
subplot(3,1,3)
plot(ts,zddot_traj)
title('Desired Acceleration Trajectory')
xlabel('time')
ylabel('z\_ddot(t)')
xlim([0,65])
ylim([-0.4,0.4])

%% plot individual trajectories
% figure()
% title('X1: p0 to p1')
% subplot(3,1,1)
% plot(t0:0.1:t1,subs(Q(1,1),t,t0:0.1:t1));
% subplot(3,1,2)
% plot(t0:0.1:t1,subs(Q_dot(1,1),t,t0:0.1:t1));
% subplot(3,1,3)
% plot(t0:0.1:t1,subs(Q_ddot(1,1),t,t0:0.1:t1));
% 
% figure()
% title('X2: p1 to p2')
% subplot(3,1,1)
% plot(t1:0.1:t2,subs(Q(2,1),t,t1:0.1:t2));
% subplot(3,1,2)
% plot(t1:0.1:t2,subs(Q_dot(2,1),t,t1:0.1:t2));
% subplot(3,1,3)
% plot(t1:0.1:t2,subs(Q_ddot(2,1),t,t1:0.1:t2));
% 
% figure()
% title('X3: p2 to p3')
% subplot(3,1,1)
% plot(t2:0.1:t3,subs(Q(3,1),t,t2:0.1:t3));
% subplot(3,1,2)
% plot(t2:0.1:t3,subs(Q_dot(3,1),t,t2:0.1:t3));
% subplot(3,1,3)
% plot(t2:0.1:t3,subs(Q_ddot(3,1),t,t2:0.1:t3));
% 
% figure()
% title('X4: p3 to p4')
% subplot(3,1,1)
% plot(t3:0.1:t4,subs(Q(4,1),t,t3:0.1:t4));
% subplot(3,1,2)
% plot(t3:0.1:t4,subs(Q_dot(4,1),t,t3:0.1:t4));
% subplot(3,1,3)
% plot(t3:0.1:t4,subs(Q_ddot(4,1),t,t3:0.1:t4));
% 
% figure()
% title('X5: p4 to p5')
% subplot(3,1,1)
% plot(t4:0.1:t5,subs(Q(5,1),t,t4:0.1:t5));
% subplot(3,1,2)
% plot(t4:0.1:t5,subs(Q_dot(5,1),t,t4:0.1:t5));
% subplot(3,1,3)
% plot(t4:0.1:t5,subs(Q_ddot(5,1),t,t4:0.1:t5));

