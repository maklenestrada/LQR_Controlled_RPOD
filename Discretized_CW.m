function [A,B] = Discretized_CW(delta_t,n)
%Inputs:
%   delta_t
%   n: Mean Motion
%Outputs:
%   A: State Transition Matrix
%   B: Control Input Matrix

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

A = [R11 R12 R13 R14 R15 R16;
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

B = [b11, b12, b13;
        b21, b22, b23;
        b31, b32, b33;
        b41, b42, b43;
        b51, b52, b53;
        b61, b62, b63];
end