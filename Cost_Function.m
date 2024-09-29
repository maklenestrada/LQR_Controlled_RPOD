function J = Cost_Function(U,Xo,A,B,Q,R,S,ts)
%Inputs:
%   U: Control Vector 
%   Xo: Initial Conditions 
%   A: State Transition Matrix
%   B: Control Input Matrix
%   Q: State Cost Matrix
%   R: Control Effort Matrix
%   S: Terminal Cost
%   ts: Time Steps

%Output:
%   J: Total Cost

%% Initialize trajectory vector
x = zeros(6,ts);
x(:,1) = Xo; %Set Initial value in array

%Initialize the Cost J
J = 0;

%Main Loop
for i = 1:(ts-1)
    %Control Input at current time step
    u_i = U(:,i);

    %X at current time step
    x_i = x(:,i+1);

    %Compute the cost
    J_i = (x_i' * Q * x_i) + (u_i' * R * u_i);
    J = J + J_i;

    %Update State 
    x(:,i+1) = A*x(:,i+1) + B*u_i;
end

% Adding Terminal Cost S (at the end)
    J = J + (x(:,end)' * S * x(:,end));
end