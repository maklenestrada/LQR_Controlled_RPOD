function U_Check = U_Constraint_Check(U,ux_min,ux_max,uy_min,uy_max,uz_min,uz_max,ts)
%Inputs:
%   U: Control Vector
%   U_min: Lower Bound on Control Saturation Limit
%   U_max: Upper Bound on Control Saturation Limit
%   ts: Time Steps
%Output:
%   U_Min: Boolean Matrix Checking Lower Bound U_Min - U 
%   U_Max: Boolean Matrix Checking Upper Bound U - U_Max

%Defining U_Mix_Con and U_Max_Con
U_Min_Con = zeros(3,ts);
U_Max_Con = zeros(3,ts);

%Defining U_min and U_max for boolean checl
%Seperating U into X,Y and Z components
U_x = U(1,:);
U_y = U(2,:);
U_z = U(3,:);

%
%% Check U_Min - U & U - U_Max
for i = 1:ts
   %Lower Bound Check
   %X-Comp
   U_Min_Con(1,i) = ux_min - U_x(i);
   %Y-Comp
   U_Min_Con(2,i) = uy_min - U_y(i);
   %Z-Comp
   U_Min_Con(3,i) = uz_min - U_z(i);

   %Upper Bound Check
   %X-Comp
   U_Max_Con(1,i) = U_x(i) - ux_max;
   %Y-Comp
   U_Max_Con(2,i) = U_y(i) - uy_max;
   %Z-Comp
   U_Max_Con(3,i) = U_z(i) - uz_max;
end

% %Boolean Check
% for i = 1:ts
%     %Lower Bound Check
%     %X-Comp
%     if U_Min_Con(1,i) < 0
%         U_Min(1,i) = 1;
%     else 
%         U_Min(1,i) = 0;
%     end
%     %Y-Comp
%     if U_Min_Con(2,i) < 0
%         U_Min(2,i) = 1;
%     else 
%         U_Min(2,i) = 0;
%     end
%     %Z-Comp
%     if U_Min_Con(3,i) < 0
%         U_Min(3,i) = 1;
%     else 
%         U_Min(3,i) = 0;
%     end
% 
%     %Upper Bound Check
%     %X-Comp
%     if U_Max_Con(1,i) < 0 
%         U_Max(1,i) = 1;
%     else
%         U_Max(1,i) = 0;
%     end
%     %Y-Comp
%     if U_Max_Con(2,i) < 0 
%         U_Max(2,i) = 1;
%     else
%         U_Max(2,i) = 0;
%     end
%     %Z-Comp
%     if U_Max_Con(3,i) < 0 
%         U_Max(3,i) = 1;
%     else
%         U_Max(3,i) = 0;
%     end
% end

U_Check = [U_Min_Con(1,:)';U_Max_Con(1,:)';U_Min_Con(2,:)';U_Max_Con(2,:)';U_Min_Con(3,:)';U_Max_Con(3,:)'];
end