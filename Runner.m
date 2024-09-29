%Maklen A. Estrada
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

%% Get Control Vector 
[K,S,E] = dlqr(A,B,Q,R);

%Initialize trajectory array 
x_traj = zeros(6,ts);
x_traj(:,1) = Xo; %Set Initial value in array

%Initialize Control array
U = zeros(3,ts);

%Control Saturation Limits (Randomly Chosen for now)
ux_min = .0005;
ux_max = 1e27;
uy_min = .0005;
uy_max = 1e27;
uz_min = .0005;
uz_max = 1e27;

%Main Loop
for i = 1:(ts-1)
    %Control Input
    u = -K * x_traj(:,i);

    %Control Input Array
    U(:,i) = u;
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

%% Cost Function
J = Cost_Function(U,Xo,A,B,Q,R,S,ts);

%% Control Saturation Limit Check
U_Check = U_Constraint_Check(U,ux_min,ux_max,uy_min,uy_max,uz_min,uz_max,ts);

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

%Trajectory plot
figure;
plot3(x_pos, y_pos, z_pos, 'LineWidth', 1.5);
xlabel('X Position ');
ylabel('Y Position ');
zlabel('Z Position ');
title('3D Trajectory');
grid on;
axis equal; 