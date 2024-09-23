function J = Calculate_Cost_Function(U,Xo,A,B,Q,R,ts)
%Inputs:
%   U: Control Vector 
%   Xo: Initial Conditions 
%   A: State Transition Matrix
%   B: Control Input Matrix
%   Q: State Cost Matrix
%   R: Control Effort Matrix
%   ts: Time Steps

%Output:
%   J: Total Cost
%Solve Ricatti Equation to get Ricatti Matrix S
[K,S,E] = dlqr(A,B,Q,R);

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
    J_i = (x_i' * Q * x_i) + (u_i' * R * u_i) + (x_i' * S * x_i);
    J = J + J_i;

    %Update State 
    x(:,i+1) = A*x(:,i+1) + B*u_i;
end
end