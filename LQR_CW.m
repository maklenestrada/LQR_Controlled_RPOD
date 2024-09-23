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
R11 = 4 - 3 * cos( n * delta_t);
R12 = 0;
R13 = 0;
R14 = (1/n) * sin( n * delta_t);
R15 = (2/n) * ( 1 - cos( n * delta_t));
R16 = 0;
R21 = 6 * (sin( n * delta_t) - n * delta_t);
R22 = 1;
R23 = 0;
R24 = -(2/n) * ( 1 - cos( n * delta_t));
R25 = (1/n) * (4 * sin( n * delta_t) - 3 * n * delta_t);
R26 = 0;
R31 = 0;
R32 = 0;
R33 = cos( n * delta_t);
R34 = 0;
R35 = 0; 
R36 = (1/n) * sin ( n * delta_t);
R41 = 3 * n * sin( n * delta_t);
R42 = 0;
R43 = 0;
R44 = cos( n * delta_t);
R45 = 2 * sin( n * delta_t);
R46 = 0;
R51 = - 6 * n * ( 1 - cos( n * delta_t));
R52 = 0;
R53 = 0;
R54 = -2 * sin( n * delta_t);
R55 = 4 * cos( n * delta_t) - 3;
R56 = 0;
R61 = 0;
R62 = 0;
R63 = -n * sin( n * delta_t);
R64 = 0;
R65 = 0;
R66 = cos( n * delta_t);

Abar = [R11 R12 R13 R14 R15 R16;
        R21 R22 R23 R24 R25 R26;
        R31 R32 R33 R34 R35 R36;
        R41 R42 R43 R44 R45 R46;
        R51 R52 R53 R54 R55 R56;
        R61 R62 R63 R64 R65 R66];

b11 = (1/n) * sin( n * delta_t);
b12 = (2/n) * ( 1 - cos( n - delta_t));
b13 = 0;
b21 = -(2/n^2) * ( n * delta_t - sin( n * delta_t));
b22 = (4/n^2) * (1 - cos( n * delta_t)) - (3/2) * delta_t^2;
b23 = 0;
b31 = 0;
b32 = 0;
b33 = (1/n^2) * ( 1 - cos( n * delta_t));
b41 = (1/n) * sin( n * delta_t);
b42 = (2/n) * ( 1 - cos( n * delta_t));
b43 = 0;
b51 = -(2/n) * ( 1 - cos( n * delta_t));
b52 = (4/n) * sin( n * delta_t) - 3 * delta_t;
b53 = 0;
b61 = 0;
b62 = 0;
b63 = (1/n) * sin( n * delta_t);

Bbar = [b11, b12, b13;
        b21, b22, b23;
        b31, b32, b33;
        b41, b42, b43;
        b51, b52, b53;
        b61, b62, b63];

%% LQR 
% Q Matrix Guess
Q = [10 0 0 0 0 0;
     0 10 0 0 0 0;
     0 0 10 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1]*1e-3;

% R Matrix Guess
R = [1 0 0;
     0 1 0;
     0 0 1]*1e6;

[K,S,E] = dlqr(Abar,Bbar,Q,R);
% K = Full State Feedback Gain Matrix
% S = Solution to Algebraic Ricatti Equation
% E = eig(A- Bk)

%Initial xbar value
%xbar = [x u]'
xbar_o = [1 1 1 0 0 0]*100;

%Initial Control
%u_o = [1 0 0];

%Initialize trajectory array 
x_traj = zeros(6,ts);
x_traj(:,1) = xbar_o; %Set Initial value in array

%Initialize Control array
u_vec = zeros(3,ts);

%Gain Value Saturation Limits (Randomly Chosen for now)
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
    u_vec(:,i) = u;
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
    x_traj(:,i+1) = Abar * x_traj(:,i) + Bbar * u;
end

x_pos = x_traj(1,:);
y_pos = x_traj(2,:);
z_pos = x_traj(3,:);
%Plot
t = linspace(t_i,t_f,ts);
figure
hold on
grid on
plot(x_pos);
plot(y_pos);
plot(z_pos);
figure
plot3(x_pos,y_pos,z_pos);

figure
grid on
plot(t,u_vec,"LineWidth",2)
xlabel("Time")
ylabel("Control Input")