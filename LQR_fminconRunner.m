%Maklen A. Estrada
%This script is to get optimal control input for LQR controller modeling CW equations
%using fmincon
clear all
clc
close all

%% Time 
t_i = 0;    
t_f = 60*20;
ts = 50;    %Time Steps
delta_t = (t_f - t_i)/ts;

%% Mean Motion 
mu = 3.986e14; % Gravitational parameter of Earth (m^3/s^2)
Re = 6371e3;   % Radius of Earth (m)
h = 400e3;     % Altitude of the LEO orbit (m) (e.g., 400 km for ISS)
r = Re + h;    % Orbit Radius (m)
n = sqrt(mu / r^3); % Mean Motion

%%  Discretized CW Plant Dynamics 
[A,B] = Discretized_CW(delta_t,n);

%% LQR Matricies
%State Cost Matrix
% Q Matrix Guess
Q = [10 0 0 0 0 0;
     0 10 0 0 0 0;
     0 0 10 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1]*1e-3;

%Control Effort Cost Matrix
% R Matrix Guess
R = [1 0 0;
     0 1 0;
     0 0 1]*1e6;

%Initial Condition 
Xo = [1 1 1 0 0 0]'*100;

%% Get Initial "Guess" for Control Vector 
[K,S,E] = dlqr(A,B,Q,R);

%Initialize trajectory array 
x_traj = zeros(6,ts);
x_traj(:,1) = Xo; %Set Initial value in array

%Initialize Control array
Uo = zeros(3,ts);

%Control Saturation Limits (Randomly Chosen for now)
ux_min = .0005;
ux_max = 1e10;
uy_min = .0005;
uy_max = 1e10;
uz_min = .0005;
uz_max = 1e10;

%Main Loop
for i = 1:(ts-1)
    %Control Input
    u = -K * x_traj(:,i);

    %Control Input Array
    Uo(:,i) = u;
    u_n = abs(u);

    %Control Saturation Limits 
    % X-Comp
    if u_n(1) > ux_max    %Upper Bound
        u(1) = ux_max;
    end
    if u_n(1) < ux_min    %Lower Bound
        u(1) = ux_min;
    end
    % Y-Comp
    if u_n(2) > uy_max    %Upper Bound
        u(2) = uy_max;
    end
    if u_n(2) < uy_min    %Lower Bound
        u(2) = uy_min;
    end
    % Z-Comp
    if u_n(3) > uz_max    %Upper Bound
        u(3) = uz_max;
    end
    if u_n(3) < uz_min    %Lower Bound
        u(3) = uz_min;
    end

    %Update State
    x_traj(:,i+1) = A * x_traj(:,i) + B * u;
end


%Lower & Upper Bound for control input
lb = [ux_min*ones(1,ts);uy_min*ones(1,ts);uz_min*ones(1,ts)];

%Upper Bound for control input
ub = [ux_max*ones(1,ts);uy_max*ones(1,ts);uz_max*ones(1,ts)];

%Call fmincon to minimize control 
[U_Opt, fval, exitflag, output] = fmincon(@(U) Cost_Function(U,Xo,A,B,Q,R,S,ts),Uo,[],[],[],[],lb,ub, @(U) U_Constraint_Check(U,ux_min,ux_max,uy_min,uy_max,uz_min,uz_max,ts));

%% Simulate System with Optimal Control
%Initialize trajectory array 
x_traj_opt = zeros(6,ts);
x_traj_opt(:,1) = Xo; %Set Initial value in array

for i = 1:(ts-1)
    x_traj_opt(:,i+1) = A * x_traj_opt(:,i) + B * U_Opt(:,i);
end

%% Plots 
x_pos = x_traj(1,:);
y_pos = x_traj(2,:);
z_pos = x_traj(3,:);

%Time vector
t = linspace(t_i,t_f,ts);

%Position Plots
figure;
subplot(3,1,1);
plot(t, x_pos);
title('X Position');
xlabel('Time');
ylabel('X');
grid on;

subplot(3,1,2);
plot(t, y_pos);
title('Y Position');
xlabel('Time');
ylabel('Y');
grid on;

subplot(3,1,3);
plot(t, z_pos);
title('Z Position');
xlabel('Time');
ylabel('Z');
grid on;

%Control Plot 
figure;

ux = Uo(1,:);
uy = Uo(2,:);
uz = Uo(3,:);

subplot(3,1,1);
plot(t, ux,'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('u_{x}');
grid on;
subplot(3,1,2); 
plot(t, uy,'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('u_{y}');
grid on;
subplot(3,1,3); 
plot(t, uz ,'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('u_{z}');
grid on;

%Trajectory plot
figure;
plot3(x_pos, y_pos, z_pos, 'LineWidth', 1.5);
xlabel('X Position ');
ylabel('Y Position ');
zlabel('Z Position ');
title('3D Trajectory');
grid on;
axis equal; 

%% Optimal Plots 
x_pos = x_traj_opt(1,:);
y_pos = x_traj_opt(2,:);
z_pos = x_traj_opt(3,:);

%Time vector
t = linspace(t_i,t_f,ts);

%Position Plots
figure;
subplot(3,1,1);
plot(t, x_pos);
title('Optimal X Position');
xlabel('Time');
ylabel('X');
grid on;

subplot(3,1,2);
plot(t, y_pos);
title('Optimal Y Position');
xlabel('Time');
ylabel('Y');
grid on;

subplot(3,1,3);
plot(t, z_pos);
title('Optimal Z Position');
xlabel('Time');
ylabel('Z');
grid on;

%Optimal Control Plot 
figure;

ux_opt = U_Opt(1,:);
uy_opt = U_Opt(2,:);
uz_opt = U_Opt(3,:);

subplot(3,1,1);
plot(t, ux_opt,'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('u_{x_{opt}}');
grid on;
subplot(3,1,2); 
plot(t, uy_opt,'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('u_{y_{opt}}');
grid on;
subplot(3,1,3); 
plot(t, uz_opt ,'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('u_{z_{opt}}');
grid on;

%Trajectory plot
figure;
plot3(x_pos, y_pos, z_pos, 'LineWidth', 1.5);
xlabel('X Position ');
ylabel('Y Position ');
zlabel('Z Position ');
title('Optimal 3D Trajectory');
grid on;
axis equal; 